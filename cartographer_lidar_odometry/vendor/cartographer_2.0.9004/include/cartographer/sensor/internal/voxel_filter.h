// Vendored public declaration from Cartographer 2.0.9004 (Apache-2.0).
#ifndef CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
namespace cartographer { namespace sensor {
std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, float resolution);
PointCloud VoxelFilter(const PointCloud& point_cloud, float resolution);
TimedPointCloud VoxelFilter(const TimedPointCloud& point_cloud, float resolution);
std::vector<TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<TimedPointCloudOriginData::RangeMeasurement>&, float);
proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);
PointCloud AdaptiveVoxelFilter(
    const PointCloud&, const proto::AdaptiveVoxelFilterOptions&);
}}  // namespace cartographer::sensor
#endif
