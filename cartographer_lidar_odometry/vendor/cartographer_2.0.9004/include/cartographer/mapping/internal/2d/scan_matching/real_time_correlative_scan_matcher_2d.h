// Vendored public declaration from Cartographer 2.0.9004 (Apache-2.0).
#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
#include <vector>
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"
#include "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"
namespace cartographer { namespace mapping { namespace scan_matching {
class RealTimeCorrelativeScanMatcher2D {
 public:
  explicit RealTimeCorrelativeScanMatcher2D(
      const proto::RealTimeCorrelativeScanMatcherOptions& options);
  RealTimeCorrelativeScanMatcher2D(const RealTimeCorrelativeScanMatcher2D&) = delete;
  RealTimeCorrelativeScanMatcher2D& operator=(const RealTimeCorrelativeScanMatcher2D&) = delete;
  double Match(const transform::Rigid2d& initial_pose_estimate,
               const sensor::PointCloud& point_cloud, const Grid2D& grid,
               transform::Rigid2d* pose_estimate) const;
  void ScoreCandidates(const Grid2D& grid,
                       const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* candidates) const;
 private:
  std::vector<Candidate2D> GenerateExhaustiveSearchCandidates(
      const SearchParameters&) const;
  const proto::RealTimeCorrelativeScanMatcherOptions options_;
};
}}}  // namespace cartographer::mapping::scan_matching
#endif
