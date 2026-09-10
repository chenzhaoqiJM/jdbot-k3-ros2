#include "scan_planner_nav2_controller/scan_planner_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace scan_planner_nav2_controller
{
namespace
{
constexpr double kEpsilon = 1.0e-6;

struct SearchNode
{
  unsigned int index;
  double score;
  bool operator<(const SearchNode & rhs) const {return score > rhs.score;}
};

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}
}  // namespace

template<typename T>
T ScanPlannerController::parameter(const std::string & key, const T & default_value)
{
  auto node = node_.lock();
  const auto full_name = plugin_name_ + "." + key;
  if (!node->has_parameter(full_name)) {
    node->declare_parameter<T>(full_name, default_value);
  }
  return node->get_parameter(full_name).get_value<T>();
}

void ScanPlannerController::configure(
  const LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("SCAN controller lifecycle node expired during configure");
  }

  planning_horizon_ = parameter("planning_horizon", planning_horizon_);
  control_point_distance_ = parameter("control_point_distance", control_point_distance_);
  spline_dt_ = parameter("spline_dt", spline_dt_);
  replan_period_ = parameter("replan_period", replan_period_);
  lookahead_time_ = parameter("lookahead_time", lookahead_time_);
  transform_tolerance_ = parameter("transform_tolerance", transform_tolerance_);
  max_linear_velocity_ = parameter("max_linear_velocity", max_linear_velocity_);
  max_angular_velocity_ = parameter("max_angular_velocity", max_angular_velocity_);
  max_acceleration_ = parameter("max_acceleration", max_acceleration_);
  max_angular_acceleration_ = parameter("max_angular_acceleration", max_angular_acceleration_);
  min_linear_velocity_ = parameter("min_linear_velocity", min_linear_velocity_);
  position_gain_ = parameter("position_gain", position_gain_);
  yaw_gain_ = parameter("yaw_gain", yaw_gain_);
  rotate_to_heading_angle_ = parameter("rotate_to_heading_angle", rotate_to_heading_angle_);
  goal_dist_tolerance_ = parameter("goal_dist_tolerance", goal_dist_tolerance_);
  collision_clearance_ = parameter("collision_clearance", collision_clearance_);
  double_cylinder_offset_ = parameter("double_cylinder_offset", double_cylinder_offset_);
  lambda_smooth_ = parameter("lambda_smooth", lambda_smooth_);
  lambda_collision_ = parameter("lambda_collision", lambda_collision_);
  lambda_feasibility_ = parameter("lambda_feasibility", lambda_feasibility_);
  lambda_fitness_ = parameter("lambda_fitness", lambda_fitness_);
  collision_cost_threshold_ = parameter("collision_cost_threshold", collision_cost_threshold_);
  max_iterations_ = parameter("max_iterations", max_iterations_);
  allow_unknown_ = parameter("allow_unknown", allow_unknown_);

  trajectory_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "~/scan_planner_trajectory", rclcpp::SystemDefaultsQoS());
  last_plan_time_ = node->now();
  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s: horizon %.2fm, cubic B-spline dt %.2fs, vmax %.2fm/s",
    plugin_name_.c_str(), planning_horizon_, spline_dt_, max_linear_velocity_);
}

void ScanPlannerController::cleanup()
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_.poses.clear();
  trajectory_control_.clear();
  trajectory_pub_.reset();
}

void ScanPlannerController::activate()
{
  if (trajectory_pub_) {
    trajectory_pub_->on_activate();
  }
}

void ScanPlannerController::deactivate()
{
  if (trajectory_pub_) {
    trajectory_pub_->on_deactivate();
  }
}

void ScanPlannerController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("SCAN controller received an empty global plan");
  }
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
  if (auto node = node_.lock()) {
    last_plan_time_ = node->now() - rclcpp::Duration::from_seconds(replan_period_ + 0.01);
  }
}

bool ScanPlannerController::transformPlan(
  const std::string & target_frame, std::vector<Point2> & points,
  geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path plan;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plan = global_plan_;
  }
  if (plan.poses.empty()) {
    return false;
  }

  points.clear();
  points.reserve(plan.poses.size());
  try {
    for (auto pose : plan.poses) {
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = plan.header.frame_id;
      }
      geometry_msgs::msg::PoseStamped transformed;
      tf_->transform(pose, transformed, target_frame, tf2::durationFromSec(transform_tolerance_));
      points.push_back({transformed.pose.position.x, transformed.pose.position.y});
    }
    auto goal_pose = plan.poses.back();
    if (goal_pose.header.frame_id.empty()) {
      goal_pose.header.frame_id = plan.header.frame_id;
    }
    tf_->transform(goal_pose, goal, target_frame, tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & error) {
    if (auto node = node_.lock()) {
      RCLCPP_WARN(node->get_logger(), "Cannot transform global plan: %s", error.what());
    }
    return false;
  }
  return true;
}

std::vector<Point2> ScanPlannerController::makeLocalReference(
  const std::vector<Point2> & plan, const Point2 & robot) const
{
  if (plan.empty()) {
    return {};
  }
  size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan.size(); ++i) {
    const double d = norm(plan[i] - robot);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }

  std::vector<Point2> local{robot};
  double length = 0.0;
  Point2 previous = robot;
  auto * map = costmap_ros_->getCostmap();
  const unsigned int edge_margin = static_cast<unsigned int>(std::ceil(
      (collision_clearance_ + double_cylinder_offset_) / map->getResolution()));
  const auto inside_local_map = [&](const Point2 & point) {
      unsigned int mx = 0;
      unsigned int my = 0;
      return map->worldToMap(point.x, point.y, mx, my) && mx >= edge_margin && my >= edge_margin &&
             mx + edge_margin < map->getSizeInCellsX() &&
             my + edge_margin < map->getSizeInCellsY();
    };
  for (size_t i = nearest; i < plan.size(); ++i) {
    if (!inside_local_map(plan[i])) {
      break;
    }
    const double segment = norm(plan[i] - previous);
    if (length + segment > planning_horizon_) {
      const double remaining = planning_horizon_ - length;
      if (segment > kEpsilon && remaining > 0.0) {
        const Point2 horizon_point = previous + (plan[i] - previous) * (remaining / segment);
        if (inside_local_map(horizon_point)) {
          local.push_back(horizon_point);
        }
      }
      break;
    }
    if (segment > control_point_distance_ * 0.15) {
      local.push_back(plan[i]);
      length += segment;
      previous = plan[i];
    }
  }
  return local;
}

std::vector<Point2> ScanPlannerController::resample(
  const std::vector<Point2> & points, double spacing) const
{
  if (points.size() < 2) {
    return points;
  }
  std::vector<Point2> output{points.front()};
  Point2 sample = points.front();
  size_t segment_id = 1;
  double remainder = spacing;
  while (segment_id < points.size()) {
    const Point2 delta = points[segment_id] - sample;
    const double distance = norm(delta);
    if (distance + kEpsilon >= remainder) {
      sample = sample + delta * (remainder / std::max(distance, kEpsilon));
      output.push_back(sample);
      remainder = spacing;
    } else {
      remainder -= distance;
      sample = points[segment_id++];
    }
  }
  if (norm(output.back() - points.back()) > spacing * 0.20) {
    output.push_back(points.back());
  } else {
    output.back() = points.back();
  }
  while (output.size() < 4) {
    const size_t largest = output.size() - 1;
    output.insert(output.begin() + static_cast<long>(largest),
      (output[largest - 1] + output[largest]) * 0.5);
  }
  return output;
}

bool ScanPlannerController::collision(const Point2 & point, double yaw) const
{
  auto * map = costmap_ros_->getCostmap();
  const Point2 heading{std::cos(yaw), std::sin(yaw)};
  const std::array<Point2, 3> samples{
    point, point + heading * double_cylinder_offset_, point - heading * double_cylinder_offset_};
  for (const auto & sample : samples) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!map->worldToMap(sample.x, sample.y, mx, my)) {
      return true;
    }
    const auto cost = map->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      if (!allow_unknown_) {
        return true;
      }
    } else if (cost >= static_cast<unsigned char>(collision_cost_threshold_)) {
      return true;
    }
  }
  return false;
}

bool ScanPlannerController::aStar(
  const Point2 & start, const Point2 & goal, std::vector<Point2> & path) const
{
  auto * map = costmap_ros_->getCostmap();
  unsigned int sx, sy, gx, gy;
  if (!map->worldToMap(start.x, start.y, sx, sy) || !map->worldToMap(goal.x, goal.y, gx, gy)) {
    return false;
  }
  const unsigned int width = map->getSizeInCellsX();
  const unsigned int height = map->getSizeInCellsY();
  const size_t count = static_cast<size_t>(width) * height;
  const auto address = [width](unsigned int x, unsigned int y) {return y * width + x;};
  const auto heuristic = [gx, gy](unsigned int x, unsigned int y) {
      return std::hypot(static_cast<double>(gx) - x, static_cast<double>(gy) - y);
    };
  const auto valid = [&](unsigned int x, unsigned int y, double yaw) {
      double wx, wy;
      map->mapToWorld(x, y, wx, wy);
      return !collision({wx, wy}, yaw);
    };

  std::vector<double> g_score(count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(count, -1);
  std::vector<bool> closed(count, false);
  std::priority_queue<SearchNode> open;
  const unsigned int start_index = address(sx, sy);
  const unsigned int goal_index = address(gx, gy);
  g_score[start_index] = 0.0;
  open.push({start_index, heuristic(sx, sy)});

  constexpr int neighbors[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1}, {1, -1}, {1, 0}, {1, 1}};
  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = true;
    if (current.index == goal_index) {
      std::vector<Point2> reverse;
      int index = static_cast<int>(goal_index);
      while (index >= 0) {
        const unsigned int x = static_cast<unsigned int>(index) % width;
        const unsigned int y = static_cast<unsigned int>(index) / width;
        double wx, wy;
        map->mapToWorld(x, y, wx, wy);
        reverse.push_back({wx, wy});
        index = parent[static_cast<size_t>(index)];
      }
      path.assign(reverse.rbegin(), reverse.rend());
      path.front() = start;
      path.back() = goal;
      return true;
    }
    const unsigned int cx = current.index % width;
    const unsigned int cy = current.index / width;
    for (const auto & offset : neighbors) {
      const int nx_i = static_cast<int>(cx) + offset[0];
      const int ny_i = static_cast<int>(cy) + offset[1];
      if (nx_i < 1 || ny_i < 1 || nx_i >= static_cast<int>(width) - 1 ||
        ny_i >= static_cast<int>(height) - 1)
      {
        continue;
      }
      const auto nx = static_cast<unsigned int>(nx_i);
      const auto ny = static_cast<unsigned int>(ny_i);
      const auto next_index = address(nx, ny);
      if (closed[next_index]) {
        continue;
      }
      const double yaw = std::atan2(static_cast<double>(offset[1]), static_cast<double>(offset[0]));
      if (!valid(nx, ny, yaw)) {
        continue;
      }
      const double step = std::hypot(static_cast<double>(offset[0]), static_cast<double>(offset[1]));
      const auto cell_cost = map->getCost(nx, ny);
      const double cost_bias = cell_cost == nav2_costmap_2d::NO_INFORMATION ? 1.0 : cell_cost / 252.0;
      const double tentative = g_score[current.index] + step * (1.0 + cost_bias);
      if (tentative < g_score[next_index]) {
        g_score[next_index] = tentative;
        parent[next_index] = static_cast<int>(current.index);
        open.push({next_index, tentative + heuristic(nx, ny)});
      }
    }
  }
  return false;
}

bool ScanPlannerController::applyAStarGuidance(std::vector<Point2> & points) const
{
  size_t i = 1;
  while (i + 1 < points.size()) {
    const double yaw = std::atan2(points[i + 1].y - points[i - 1].y,
      points[i + 1].x - points[i - 1].x);
    if (!collision(points[i], yaw)) {
      ++i;
      continue;
    }
    const size_t begin = i - 1;
    size_t end = i + 1;
    while (end + 1 < points.size()) {
      const double end_yaw = std::atan2(points[end + 1].y - points[end - 1].y,
        points[end + 1].x - points[end - 1].x);
      if (!collision(points[end], end_yaw)) {
        break;
      }
      ++end;
    }
    if (end >= points.size() || collision(points[end], yaw)) {
      return false;
    }
    std::vector<Point2> detour;
    if (!aStar(points[begin], points[end], detour)) {
      return false;
    }
    auto replacement = resample(detour, control_point_distance_);
    points.erase(points.begin() + static_cast<long>(begin + 1), points.begin() + static_cast<long>(end));
    points.insert(points.begin() + static_cast<long>(begin + 1), replacement.begin() + 1, replacement.end() - 1);
    i = begin + replacement.size();
  }
  return true;
}

bool ScanPlannerController::nearestObstacle(
  const Point2 & point, Point2 & obstacle, double & distance) const
{
  auto * map = costmap_ros_->getCostmap();
  unsigned int mx, my;
  if (!map->worldToMap(point.x, point.y, mx, my)) {
    distance = 0.0;
    obstacle = point;
    return true;
  }
  const int radius = std::max(1, static_cast<int>(std::ceil(collision_clearance_ / map->getResolution())));
  bool found = false;
  double best2 = collision_clearance_ * collision_clearance_;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= static_cast<int>(map->getSizeInCellsX()) ||
        y >= static_cast<int>(map->getSizeInCellsY()))
      {
        continue;
      }
      const auto cost = map->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      const bool occupied = cost == nav2_costmap_2d::NO_INFORMATION ? !allow_unknown_ :
        cost >= static_cast<unsigned char>(collision_cost_threshold_);
      if (!occupied) {
        continue;
      }
      double wx, wy;
      map->mapToWorld(static_cast<unsigned int>(x), static_cast<unsigned int>(y), wx, wy);
      const double d2 = (wx - point.x) * (wx - point.x) + (wy - point.y) * (wy - point.y);
      if (d2 < best2) {
        best2 = d2;
        obstacle = {wx, wy};
        found = true;
      }
    }
  }
  distance = found ? std::sqrt(best2) : collision_clearance_;
  return found;
}

double ScanPlannerController::objective(
  const std::vector<double> & x, const std::vector<Point2> & reference,
  std::vector<double> * gradient) const
{
  std::vector<Point2> q(reference.size());
  q.front() = reference.front();
  q.back() = reference.back();
  for (size_t i = 1; i + 1 < q.size(); ++i) {
    q[i] = {x[2 * (i - 1)], x[2 * (i - 1) + 1]};
  }
  std::vector<Point2> g(q.size());
  double smooth = 0.0;
  double collision_cost = 0.0;
  double feasibility = 0.0;
  double fitness = 0.0;

  for (size_t i = 1; i + 1 < q.size(); ++i) {
    const Point2 second = q[i - 1] - q[i] * 2.0 + q[i + 1];
    smooth += second.x * second.x + second.y * second.y;
    g[i - 1] = g[i - 1] + second * (2.0 * lambda_smooth_);
    g[i] = g[i] - second * (4.0 * lambda_smooth_);
    g[i + 1] = g[i + 1] + second * (2.0 * lambda_smooth_);

    const Point2 error = q[i] - reference[i];
    fitness += error.x * error.x + error.y * error.y;
    g[i] = g[i] + error * (2.0 * lambda_fitness_);

    Point2 obstacle;
    double distance;
    if (nearestObstacle(q[i], obstacle, distance) && distance < collision_clearance_) {
      const double violation = collision_clearance_ - distance;
      collision_cost += violation * violation;
      Point2 direction = q[i] - obstacle;
      const double direction_norm = norm(direction);
      if (direction_norm > kEpsilon) {
        direction = direction / direction_norm;
        g[i] = g[i] - direction * (2.0 * lambda_collision_ * violation);
      }
    }
  }

  const double max_step = max_linear_velocity_ * spline_dt_;
  const double max_second = max_acceleration_ * spline_dt_ * spline_dt_;
  for (size_t i = 0; i + 1 < q.size(); ++i) {
    const Point2 delta = q[i + 1] - q[i];
    const double length = norm(delta);
    if (length > max_step && length > kEpsilon) {
      const double violation = length - max_step;
      feasibility += violation * violation;
      const Point2 direction = delta / length;
      g[i] = g[i] - direction * (2.0 * lambda_feasibility_ * violation);
      g[i + 1] = g[i + 1] + direction * (2.0 * lambda_feasibility_ * violation);
    }
  }
  for (size_t i = 1; i + 1 < q.size(); ++i) {
    const Point2 second = q[i - 1] - q[i] * 2.0 + q[i + 1];
    const double length = norm(second);
    if (length > max_second && length > kEpsilon) {
      const double violation = length - max_second;
      feasibility += violation * violation;
      const Point2 direction = second / length;
      const Point2 weighted = direction * (2.0 * lambda_feasibility_ * violation);
      g[i - 1] = g[i - 1] + weighted;
      g[i] = g[i] - weighted * 2.0;
      g[i + 1] = g[i + 1] + weighted;
    }
  }

  if (gradient) {
    gradient->resize(x.size());
    for (size_t i = 1; i + 1 < q.size(); ++i) {
      (*gradient)[2 * (i - 1)] = g[i].x;
      (*gradient)[2 * (i - 1) + 1] = g[i].y;
    }
  }
  return lambda_smooth_ * smooth + lambda_collision_ * collision_cost +
         lambda_feasibility_ * feasibility + lambda_fitness_ * fitness;
}

double ScanPlannerController::dot(const std::vector<double> & a, const std::vector<double> & b)
{
  return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
}

bool ScanPlannerController::optimize(std::vector<Point2> & points) const
{
  if (points.size() < 3) {
    return false;
  }
  const auto reference = points;
  std::vector<double> x(2 * (points.size() - 2));
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    x[2 * (i - 1)] = points[i].x;
    x[2 * (i - 1) + 1] = points[i].y;
  }

  constexpr size_t memory = 10;
  std::vector<std::vector<double>> s_history;
  std::vector<std::vector<double>> y_history;
  std::vector<double> rho_history;
  std::vector<double> gradient;
  double value = objective(x, reference, &gradient);

  for (int iteration = 0; iteration < max_iterations_; ++iteration) {
    if (std::sqrt(dot(gradient, gradient) / std::max<size_t>(1, gradient.size())) < 1.0e-3) {
      break;
    }
    std::vector<double> direction = gradient;
    std::vector<double> alpha(s_history.size(), 0.0);
    for (size_t reverse = s_history.size(); reverse-- > 0;) {
      alpha[reverse] = rho_history[reverse] * dot(s_history[reverse], direction);
      for (size_t j = 0; j < direction.size(); ++j) {
        direction[j] -= alpha[reverse] * y_history[reverse][j];
      }
    }
    if (!s_history.empty()) {
      const auto & last_s = s_history.back();
      const auto & last_y = y_history.back();
      const double scale = dot(last_s, last_y) / std::max(dot(last_y, last_y), kEpsilon);
      for (auto & item : direction) {
        item *= scale;
      }
    }
    for (size_t i = 0; i < s_history.size(); ++i) {
      const double beta = rho_history[i] * dot(y_history[i], direction);
      for (size_t j = 0; j < direction.size(); ++j) {
        direction[j] += s_history[i][j] * (alpha[i] - beta);
      }
    }
    for (auto & item : direction) {
      item = -item;
    }
    if (dot(direction, gradient) >= 0.0) {
      for (size_t j = 0; j < direction.size(); ++j) {
        direction[j] = -gradient[j];
      }
    }

    double step = 1.0;
    const double slope = dot(gradient, direction);
    std::vector<double> next_x(x.size());
    std::vector<double> next_gradient;
    double next_value = value;
    bool accepted = false;
    for (int line_search = 0; line_search < 18; ++line_search) {
      for (size_t j = 0; j < x.size(); ++j) {
        next_x[j] = x[j] + step * direction[j];
      }
      next_value = objective(next_x, reference, &next_gradient);
      if (std::isfinite(next_value) && next_value <= value + 1.0e-4 * step * slope) {
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) {
      break;
    }

    std::vector<double> s(x.size());
    std::vector<double> y(x.size());
    for (size_t j = 0; j < x.size(); ++j) {
      s[j] = next_x[j] - x[j];
      y[j] = next_gradient[j] - gradient[j];
    }
    const double sy = dot(s, y);
    if (sy > 1.0e-10) {
      if (s_history.size() == memory) {
        s_history.erase(s_history.begin());
        y_history.erase(y_history.begin());
        rho_history.erase(rho_history.begin());
      }
      s_history.push_back(std::move(s));
      y_history.push_back(std::move(y));
      rho_history.push_back(1.0 / sy);
    }
    x.swap(next_x);
    gradient.swap(next_gradient);
    value = next_value;
  }

  for (size_t i = 1; i + 1 < points.size(); ++i) {
    points[i] = {x[2 * (i - 1)], x[2 * (i - 1) + 1]};
    const double yaw = std::atan2(points[i + 1].y - points[i - 1].y,
      points[i + 1].x - points[i - 1].x);
    if (collision(points[i], yaw)) {
      return false;
    }
  }
  return true;
}

std::vector<Point2> ScanPlannerController::clampControlPoints(
  const std::vector<Point2> & points) const
{
  std::vector<Point2> control;
  control.reserve(points.size() + 4);
  control.insert(control.end(), 3, points.front());
  if (points.size() > 2) {
    control.insert(control.end(), points.begin() + 1, points.end() - 1);
  }
  control.insert(control.end(), 3, points.back());
  return control;
}

Point2 ScanPlannerController::evaluateBspline(
  const std::vector<Point2> & control, double u) const
{
  const size_t segments = control.size() - 3;
  u = clampValue(u, 0.0, static_cast<double>(segments) - 1.0e-9);
  const size_t i = std::min(static_cast<size_t>(std::floor(u)), segments - 1);
  const double t = u - static_cast<double>(i);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double b0 = (1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0;
  const double b1 = (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0;
  const double b2 = (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0;
  const double b3 = t3 / 6.0;
  return control[i] * b0 + control[i + 1] * b1 + control[i + 2] * b2 + control[i + 3] * b3;
}

Point2 ScanPlannerController::evaluateBsplineVelocity(
  const std::vector<Point2> & control, double u) const
{
  const size_t segments = control.size() - 3;
  u = clampValue(u, 0.0, static_cast<double>(segments) - 1.0e-9);
  const size_t i = std::min(static_cast<size_t>(std::floor(u)), segments - 1);
  const double t = u - static_cast<double>(i);
  const double db0 = -0.5 * (1.0 - t) * (1.0 - t);
  const double db1 = 1.5 * t * t - 2.0 * t;
  const double db2 = -1.5 * t * t + t + 0.5;
  const double db3 = 0.5 * t * t;
  return (control[i] * db0 + control[i + 1] * db1 +
         control[i + 2] * db2 + control[i + 3] * db3) / spline_dt_;
}

void ScanPlannerController::publishTrajectory(
  const std::vector<Point2> & control, const std::string & frame)
{
  if (!trajectory_pub_ || !trajectory_pub_->is_activated() || control.size() < 4) {
    return;
  }
  auto node = node_.lock();
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = node->now();
  const double end = static_cast<double>(control.size() - 3);
  for (double u = 0.0; u < end; u += 0.10) {
    const auto point = evaluateBspline(control, u);
    const auto tangent = evaluateBsplineVelocity(control, u);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.z = std::sin(std::atan2(tangent.y, tangent.x) * 0.5);
    pose.pose.orientation.w = std::cos(std::atan2(tangent.y, tangent.x) * 0.5);
    path.poses.push_back(pose);
  }
  trajectory_pub_->publish(path);
}

geometry_msgs::msg::TwistStamped ScanPlannerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  auto node = node_.lock();
  if (!node || !costmap_ros_) {
    throw nav2_core::PlannerException("SCAN controller is not configured");
  }
  const std::string local_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::PoseStamped local_pose;
  try {
    tf_->transform(pose, local_pose, local_frame, tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & error) {
    throw nav2_core::PlannerException(std::string("Cannot transform robot pose: ") + error.what());
  }

  std::vector<Point2> transformed_plan;
  geometry_msgs::msg::PoseStamped goal;
  if (!transformPlan(local_frame, transformed_plan, goal)) {
    throw nav2_core::PlannerException("Cannot transform SCAN controller global plan");
  }
  const Point2 robot{local_pose.pose.position.x, local_pose.pose.position.y};
  const Point2 goal_point{goal.pose.position.x, goal.pose.position.y};
  const double robot_yaw = tf2::getYaw(local_pose.pose.orientation);
  const double goal_distance = norm(goal_point - robot);

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node->now();
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  if (goal_distance <= goal_dist_tolerance_) {
    const double yaw_error = normalizeAngle(tf2::getYaw(goal.pose.orientation) - robot_yaw);
    command.twist.angular.z = clampValue(yaw_gain_ * yaw_error,
      -max_angular_velocity_, max_angular_velocity_);
    return command;
  }

  const bool need_replan = trajectory_control_.empty() ||
    (node->now() - last_plan_time_).seconds() >= replan_period_;
  if (need_replan) {
    auto local = makeLocalReference(transformed_plan, robot);
    local = resample(local, control_point_distance_);
    if (local.size() < 4) {
      throw nav2_core::PlannerException("Global plan has no usable local segment");
    }
    auto * mutex = costmap_ros_->getCostmap()->getMutex();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*mutex);
    const bool plan_success = applyAStarGuidance(local) && optimize(local);
    last_plan_time_ = node->now();
    if (!plan_success) {
      if (trajectory_control_.empty()) {
        throw nav2_core::PlannerException("SCAN local B-spline optimization found no collision-free path");
      }
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "SCAN replan failed; continuing the last collision-free B-spline and retrying");
    } else {
      trajectory_control_ = clampControlPoints(local);
      publishTrajectory(trajectory_control_, local_frame);
    }
  }

  const double trajectory_end = static_cast<double>(trajectory_control_.size() - 3);
  double nearest_u = 0.0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (double u = 0.0; u < trajectory_end; u += 0.10) {
    const double distance = norm(evaluateBspline(trajectory_control_, u) - robot);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_u = u;
    }
  }
  const double lookahead_u = std::min(
    nearest_u + lookahead_time_ / spline_dt_, trajectory_end - 1.0e-9);
  const Point2 target = evaluateBspline(trajectory_control_, lookahead_u);
  Point2 desired_velocity = evaluateBsplineVelocity(trajectory_control_, lookahead_u) +
    (target - robot) * position_gain_;
  double speed = norm(desired_velocity);
  const double max_speed = max_linear_velocity_ * speed_limit_ratio_;
  if (speed > max_speed && speed > kEpsilon) {
    desired_velocity = desired_velocity * (max_speed / speed);
    speed = max_speed;
  }
  if (speed < min_linear_velocity_ && goal_distance > goal_dist_tolerance_ && speed > kEpsilon) {
    desired_velocity = desired_velocity * (min_linear_velocity_ / speed);
    speed = min_linear_velocity_;
  }

  const double desired_yaw = std::atan2(desired_velocity.y, desired_velocity.x);
  const double yaw_error = normalizeAngle(desired_yaw - robot_yaw);
  const double angular_target = clampValue(yaw_gain_ * yaw_error,
    -max_angular_velocity_, max_angular_velocity_);
  const double angular_delta = max_angular_acceleration_ / 20.0;
  command.twist.angular.z = clampValue(
    angular_target, velocity.angular.z - angular_delta, velocity.angular.z + angular_delta);
  if (std::abs(yaw_error) < rotate_to_heading_angle_) {
    const double forward = std::cos(robot_yaw) * desired_velocity.x +
      std::sin(robot_yaw) * desired_velocity.y;
    const double acceleration_delta = max_acceleration_ / 20.0;
    command.twist.linear.x = clampValue(
      forward, velocity.linear.x - acceleration_delta, velocity.linear.x + acceleration_delta);
  }
  return command;
}

void ScanPlannerController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    speed_limit_ratio_ = 1.0;
  } else if (percentage) {
    speed_limit_ratio_ = clampValue(speed_limit / 100.0, 0.0, 1.0);
  } else {
    speed_limit_ratio_ = clampValue(speed_limit / std::max(max_linear_velocity_, kEpsilon), 0.0, 1.0);
  }
}

double ScanPlannerController::norm(const Point2 & point)
{
  return std::hypot(point.x, point.y);
}

double ScanPlannerController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace scan_planner_nav2_controller

PLUGINLIB_EXPORT_CLASS(
  scan_planner_nav2_controller::ScanPlannerController,
  nav2_core::Controller)
