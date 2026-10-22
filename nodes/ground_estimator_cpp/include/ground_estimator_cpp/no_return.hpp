// No-return areas (design D1, D5): fixed-frame cells where the RGB sees a colour that is not the floor's and the ToF
// returns nothing (the black ramp). Pure core code, no ROS node wiring. The tracker keeps no points and outputs only
// caution warning areas (kind no_return, source rgb, no height, no slope), so it can never make anything lethal.
#ifndef GROUND_ESTIMATOR_CPP__NO_RETURN_HPP_
#define GROUND_ESTIMATOR_CPP__NO_RETURN_HPP_

#include <Eigen/Geometry>
#include <deque>
#include <map>
#include <vector>

#include "ground_estimator_cpp/cell_class.hpp"
#include "ground_estimator_cpp/colour.hpp"
#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/guidance.hpp"
#include "ground_estimator_cpp/traversability.hpp"

namespace ground_estimator_cpp
{

constexpr const char * KIND_NO_RETURN = "no_return";
constexpr const char * SOURCE_RGB = "rgb";

// Per-cell no-return hysteresis on the temporal grid (temporal.cell_size_m, the CellMedianFilter / CellClassFilter
// cell keys). The node calls observe() once per guidance frame and reset() with the cell filters.
class NoReturnTracker
{
public:
  // @param cfg the no-return settings (cell_min_samples, cell_min_share, window_frames, enter_frames, min_cells,
  //        clearance_cells; enabled, min_range_m and tof_margin_px belong to the caller and to sample_guidance).
  // @param cell_size_m edge of a fixed-frame cell in metres (temporal.cell_size_m), > 0.
  // @param floor_band_m half thickness of the floor band (fit.distance_m): depth samples with |height| <= it count
  //        as floor returns of their cell.
  // @throws std::invalid_argument on a non-positive cell size or a negative floor band.
  NoReturnTracker(const NoReturnConfig & cfg, double cell_size_m, double floor_band_m);

  // Judge one guidance frame and vote every tracked cell. A cell counts its no-return samples and its floor-band depth
  // samples; a no-return sample mismatches when the colour model is ready and its Mahalanobis distance exceeds
  // colour_cfg.max_mahalanobis. The cell is a no-return cell of this frame when its count is >= cell_min_samples, its
  // mismatching share of the count is >= cell_min_share, and no blocked cell lies within clearance_cells (Chebyshev
  // distance) of it. With a model that is not ready no cell is a no-return cell.
  // @param samples_base the frame's samples in base_link (sample_guidance); non-finite points are skipped.
  // @param colour the floor colour model.
  // @param colour_cfg supplies max_mahalanobis.
  // @param fixed_T_base pose of base_link in the fixed frame at the frame's stamp.
  // @param blocked_cells fixed-frame cells already seen by depth (the stable lethal and stable warning ToF cells,
  //        CellClassFilter::blocking_cells), whose occlusion shadows are not no-return cells.
  void observe(
    const GuidanceSamples & samples_base, const FloorColourModel & colour, const ColourConfig & colour_cfg,
    const Eigen::Isometry3d & fixed_T_base, const std::vector<CellKey> & blocked_cells);

  // @param base_T_fixed pose of the fixed frame in base_link (the inverse of the current fixed_T_base).
  // @return one area per 8-connected component of stable no-return cells with >= min_cells cells, ordered by the
  //         component's smallest cell key: kind no_return, source rgb, the cell outline (cell_outline) in base_link,
  //         height_m and slope_deg nullopt.
  std::vector<WarningArea> areas(const Eigen::Isometry3d & base_T_fixed) const;

  // @return the cells whose stable state is no-return, in key order.
  std::vector<CellKey> stable_cells() const;

  // Forget every cell.
  void reset();

private:
  struct Cell
  {
    std::deque<bool> votes;  // no-return cell in each of the last window_frames frames, oldest first
    bool stable = false;
  };

  NoReturnConfig cfg_;
  double cell_size_m_;
  double floor_band_m_;
  std::map<CellKey, Cell> cells_;
};

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__NO_RETURN_HPP_
