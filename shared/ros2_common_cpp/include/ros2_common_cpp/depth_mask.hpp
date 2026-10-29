// Depth-discontinuity ("veil") mask of a 16UC1 depth image (mm, 0 = no depth): the mixed and flying pixels a depth
// camera produces across the edge of a nearer object, between the object and the background behind it. A valid pixel
// is removed when some valid pixel of its window_px x window_px neighbourhood is nearer by more than
// max(abs_m, rel x z), z being the pixel's own depth. Only the far side of a jump is removed: the nearer object keeps
// its edge, the mixed pixels (each one behind its neighbour towards the object) and the first background pixel go.
// The window minimum is OpenCV's grey erosion; no ROS type is used. grow_px > 0 then dilates the removed region by
// that many pixels into the valid pixels.
#ifndef ROS2_COMMON_CPP__DEPTH_MASK_HPP_
#define ROS2_COMMON_CPP__DEPTH_MASK_HPP_

#include <opencv2/core.hpp>

namespace ros2_common_cpp
{

struct DepthEdgeMaskParams
{
  int window_px = 3;    // side of the square neighbourhood, odd, >= 3
  double abs_m = 0.01;  // smallest jump that removes a pixel, m (>= 0)
  double rel = 0.025;   // jump that removes a pixel per metre of its depth (>= 0); abs_m or rel must be > 0
  int grow_px = 0;      // dilation of the removed region, px (>= 0)
};

// @param params the mask parameters.
// @throws std::invalid_argument when window_px is even or below 3, abs_m, rel or grow_px is negative, or both
//   abs_m and rel are 0.
void check_depth_edge_mask_params(const DepthEdgeMaskParams & params);

// @param depth_mm CV_16UC1 depth in millimetres, 0 for no depth.
// @param params the mask parameters.
// @return CV_8UC1 of the same size, 255 where a valid pixel is removed, 0 elsewhere (missing pixels are never set).
// @throws std::invalid_argument on another image type or invalid parameters.
cv::Mat depth_edge_mask(const cv::Mat & depth_mm, const DepthEdgeMaskParams & params);

// Sets the pixels of depth_edge_mask to 0 (no depth) in place.
// @param depth_mm CV_16UC1 depth in millimetres, 0 for no depth; changed in place.
// @param params the mask parameters.
// @return the number of pixels removed.
// @throws std::invalid_argument as depth_edge_mask.
int apply_depth_edge_mask(cv::Mat & depth_mm, const DepthEdgeMaskParams & params);

}  // namespace ros2_common_cpp

#endif  // ROS2_COMMON_CPP__DEPTH_MASK_HPP_
