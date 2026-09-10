#ifndef SCAN_PLANNER_NAV2_CONTROLLER__SCAN_PLANNER_CONTROLLER_HPP_
#define SCAN_PLANNER_NAV2_CONTROLLER__SCAN_PLANNER_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace scan_planner_nav2_controller
{

struct Point2
{
  double x{0.0};
  double y{0.0};

  Point2 operator+(const Point2 & rhs) const {return {x + rhs.x, y + rhs.y};}
  Point2 operator-(const Point2 & rhs) const {return {x - rhs.x, y - rhs.y};}
  Point2 operator*(double value) const {return {x * value, y * value};}
  Point2 operator/(double value) const {return {x / value, y / value};}
};

class ScanPlannerController : public nav2_core::Controller
{
public:
  ScanPlannerController() = default;
  ~ScanPlannerController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

  template<typename T>
  T parameter(const std::string & key, const T & default_value);

  bool transformPlan(const std::string & target_frame, std::vector<Point2> & points,
    geometry_msgs::msg::PoseStamped & goal) const;
  std::vector<Point2> makeLocalReference(
    const std::vector<Point2> & plan, const Point2 & robot) const;
  std::vector<Point2> resample(const std::vector<Point2> & points, double spacing) const;
  bool applyAStarGuidance(std::vector<Point2> & points) const;
  bool aStar(const Point2 & start, const Point2 & goal, std::vector<Point2> & path) const;
  bool collision(const Point2 & point, double yaw) const;
  bool nearestObstacle(const Point2 & point, Point2 & obstacle, double & distance) const;
  bool optimize(std::vector<Point2> & points) const;
  double objective(const std::vector<double> & x, const std::vector<Point2> & reference,
    std::vector<double> * gradient) const;
  Point2 evaluateBspline(const std::vector<Point2> & control, double u) const;
  Point2 evaluateBsplineVelocity(const std::vector<Point2> & control, double u) const;
  std::vector<Point2> clampControlPoints(const std::vector<Point2> & points) const;
  void publishTrajectory(const std::vector<Point2> & control, const std::string & frame);
  static double norm(const Point2 & point);
  static double dot(const std::vector<double> & a, const std::vector<double> & b);
  static double normalizeAngle(double angle);

  LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  mutable std::mutex plan_mutex_;
  nav_msgs::msg::Path global_plan_;
  std::vector<Point2> trajectory_control_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  double planning_horizon_{4.0};
  double control_point_distance_{0.20};
  double spline_dt_{0.25};
  double replan_period_{0.20};
  double lookahead_time_{0.45};
  double transform_tolerance_{0.20};
  double max_linear_velocity_{0.60};
  double max_angular_velocity_{1.00};
  double max_acceleration_{0.50};
  double max_angular_acceleration_{1.00};
  double min_linear_velocity_{0.04};
  double position_gain_{1.50};
  double yaw_gain_{2.00};
  double rotate_to_heading_angle_{0.50};
  double goal_dist_tolerance_{0.10};
  double collision_clearance_{0.20};
  double double_cylinder_offset_{0.12};
  double lambda_smooth_{1.0};
  double lambda_collision_{2.0};
  double lambda_feasibility_{0.15};
  double lambda_fitness_{0.20};
  int collision_cost_threshold_{253};
  int max_iterations_{80};
  bool allow_unknown_{false};
  double speed_limit_ratio_{1.0};
};

}  // namespace scan_planner_nav2_controller

#endif  // SCAN_PLANNER_NAV2_CONTROLLER__SCAN_PLANNER_CONTROLLER_HPP_
