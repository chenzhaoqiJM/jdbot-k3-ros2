// Vendored public declaration from Cartographer 2.0.9004 (Apache-2.0).
#ifndef CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
#include <map>
#include <set>
#include <string>
#include <vector>
#include "cartographer/sensor/timed_point_cloud_data.h"
namespace cartographer { namespace mapping {
class RangeDataCollator {
 public:
  explicit RangeDataCollator(const std::vector<std::string>& ids)
      : expected_sensor_ids_(ids.begin(), ids.end()) {}
  sensor::TimedPointCloudOriginData AddRangeData(
      const std::string& sensor_id,
      sensor::TimedPointCloudData timed_point_cloud_data);
 private:
  sensor::TimedPointCloudOriginData CropAndMerge();
  const std::set<std::string> expected_sensor_ids_;
  std::map<std::string, sensor::TimedPointCloudData> id_to_pending_data_;
  common::Time current_start_ = common::Time::min();
  common::Time current_end_ = common::Time::min();
  constexpr static float kDefaultIntensityValue = 0.f;
};
}}  // namespace cartographer::mapping
#endif
