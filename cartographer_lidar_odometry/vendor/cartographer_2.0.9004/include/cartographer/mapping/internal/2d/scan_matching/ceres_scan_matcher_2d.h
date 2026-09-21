// Vendored public declaration from Cartographer 2.0.9004 (Apache-2.0).
#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#include <vector>
#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
namespace cartographer { namespace mapping { namespace scan_matching {
proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D& options);
  virtual ~CeresScanMatcher2D();
  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;
  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;
 private:
  const proto::CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;
};
}}}  // namespace cartographer::mapping::scan_matching
#endif
