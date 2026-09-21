// Vendored public declaration from Cartographer 2.0.9004 (Apache-2.0).
#ifndef CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"
namespace cartographer { namespace mapping {
proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);
class MotionFilter {
 public:
  explicit MotionFilter(const proto::MotionFilterOptions& options);
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);
 private:
  const proto::MotionFilterOptions options_;
  int num_total_ = 0;
  int num_different_ = 0;
  common::Time last_time_;
  transform::Rigid3d last_pose_;
};
}}  // namespace cartographer::mapping
#endif
