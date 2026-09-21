// Vendored public declaration from Cartographer 2.0.9004 (Apache-2.0).
#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
#include <vector>
#include "Eigen/Core"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"
namespace cartographer { namespace mapping { namespace scan_matching {
using DiscreteScan2D = std::vector<Eigen::Array2i>;
struct SearchParameters {
  struct LinearBounds { int min_x; int max_x; int min_y; int max_y; };
  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud& point_cloud, double resolution);
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);
  void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                   const CellLimits& cell_limits);
  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;
};
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud&, const SearchParameters&);
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits&, const std::vector<sensor::PointCloud>&,
    const Eigen::Translation2f&);
struct Candidate2D {
  Candidate2D(int init_scan_index, int init_x_index_offset,
              int init_y_index_offset, const SearchParameters& parameters)
      : scan_index(init_scan_index), x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-init_y_index_offset * parameters.resolution),
        y(-init_x_index_offset * parameters.resolution),
        orientation((scan_index - parameters.num_angular_perturbations) *
                    parameters.angular_perturbation_step_size) {}
  int scan_index = 0;
  int x_index_offset = 0;
  int y_index_offset = 0;
  double x = 0.;
  double y = 0.;
  double orientation = 0.;
  float score = 0.f;
  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};
}}}  // namespace cartographer::mapping::scan_matching
#endif
