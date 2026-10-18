#pragma once

#include <stdexcept>

namespace head_camera_cpp
{

/// Image size in pixels.
struct Size
{
  int width;
  int height;
};

/// Pixel rectangle: top-left corner plus size.
struct Rect
{
  int x;
  int y;
  int width;
  int height;
};

/**
 * Region of the full frame a zoom request covers.
 *
 * Size is (full.width / zoom) x (full.height / zoom) rounded down to even, centred on (u * full.width,
 * v * full.height) and shifted (never shrunk) to lie inside the frame. x, y, width and height are even.
 *
 * @param u Normalised horizontal point in [0, 1].
 * @param v Normalised vertical point in [0, 1].
 * @param zoom Zoom factor, >= 1.
 * @param full Full frame size.
 * @return The crop region in full-frame pixels.
 * @throws std::invalid_argument for u or v outside [0, 1] (or NaN), or zoom < 1.
 */
Rect zoom_region(double u, double v, int zoom, Size full);

/**
 * Output size of a crop: the longest side is min(region longest side, max_px), aspect kept, rounded to even and
 * never larger than the region (no upscaling).
 *
 * @param region The crop region.
 * @param max_px Maximum longest side in pixels, >= 1.
 * @return Output width and height.
 * @throws std::invalid_argument for max_px < 1 or an empty region.
 */
Size output_size(Rect region, int max_px);

/**
 * Autofocus window: a rectangle of `fraction` of the scaler crop's width and height centred on (u, v) mapped into
 * scaler_crop_max (its offset included) and clamped inside it.
 *
 * @param u Normalised horizontal point in [0, 1].
 * @param v Normalised vertical point in [0, 1].
 * @param fraction Window size as a fraction of scaler_crop_max, in (0, 1].
 * @param scaler_crop_max The sensor's maximum ScalerCrop rectangle.
 * @return The window in sensor ScalerCrop coordinates.
 * @throws std::invalid_argument for u or v outside [0, 1], fraction outside (0, 1], or an empty scaler crop.
 */
Rect af_window(double u, double v, double fraction, Rect scaler_crop_max);

}  // namespace head_camera_cpp
