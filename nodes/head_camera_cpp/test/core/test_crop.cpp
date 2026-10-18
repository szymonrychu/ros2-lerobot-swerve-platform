#include <gtest/gtest.h>

#include <stdexcept>

#include "head_camera_cpp/crop.hpp"

namespace
{

using head_camera_cpp::af_window;
using head_camera_cpp::output_size;
using head_camera_cpp::Rect;
using head_camera_cpp::Size;
using head_camera_cpp::zoom_region;

const Size FULL{4608, 2592};

void expect_rect(const Rect & r, int x, int y, int w, int h)
{
  EXPECT_EQ(r.x, x);
  EXPECT_EQ(r.y, y);
  EXPECT_EQ(r.width, w);
  EXPECT_EQ(r.height, h);
}

void expect_inside_and_even(const Rect & r, const Size & full)
{
  EXPECT_GE(r.x, 0);
  EXPECT_GE(r.y, 0);
  EXPECT_LE(r.x + r.width, full.width);
  EXPECT_LE(r.y + r.height, full.height);
  EXPECT_EQ(r.x % 2, 0);
  EXPECT_EQ(r.y % 2, 0);
  EXPECT_EQ(r.width % 2, 0);
  EXPECT_EQ(r.height % 2, 0);
}

}  // namespace

TEST(Crop, CentreRegionForEachFactor)
{
  expect_rect(zoom_region(0.5, 0.5, 2, FULL), 1152, 648, 2304, 1296);
  expect_rect(zoom_region(0.5, 0.5, 4, FULL), 1728, 972, 1152, 648);
  expect_rect(zoom_region(0.5, 0.5, 8, FULL), 2016, 1134, 576, 324);
}

TEST(Crop, CornersShiftInsideWithoutShrinking)
{
  for (const int zoom : {2, 4, 8}) {
    const int w = FULL.width / zoom;
    const int h = FULL.height / zoom;
    expect_rect(zoom_region(0.0, 0.0, zoom, FULL), 0, 0, w, h);
    expect_rect(zoom_region(1.0, 0.0, zoom, FULL), FULL.width - w, 0, w, h);
    expect_rect(zoom_region(0.0, 1.0, zoom, FULL), 0, FULL.height - h, w, h);
    expect_rect(zoom_region(1.0, 1.0, zoom, FULL), FULL.width - w, FULL.height - h, w, h);
  }
}

TEST(Crop, AllCoordinatesEvenForArbitraryPoints)
{
  for (const int zoom : {1, 2, 4, 8}) {
    for (double u = 0.0; u <= 1.0; u += 0.0937) {
      for (double v = 0.0; v <= 1.0; v += 0.0731) {
        expect_inside_and_even(zoom_region(u, v, zoom, FULL), FULL);
      }
    }
  }
  expect_inside_and_even(zoom_region(0.37, 0.61, 8, Size{1921, 1081}), Size{1921, 1081});
}

TEST(Crop, ZoomOneIsTheWholeFrame) { expect_rect(zoom_region(0.3, 0.9, 1, FULL), 0, 0, 4608, 2592); }

TEST(Crop, InvalidArgumentsThrow)
{
  EXPECT_THROW(zoom_region(-0.01, 0.5, 2, FULL), std::invalid_argument);
  EXPECT_THROW(zoom_region(1.01, 0.5, 2, FULL), std::invalid_argument);
  EXPECT_THROW(zoom_region(0.5, -0.01, 2, FULL), std::invalid_argument);
  EXPECT_THROW(zoom_region(0.5, 1.01, 2, FULL), std::invalid_argument);
  EXPECT_THROW(zoom_region(0.5, 0.5, 0, FULL), std::invalid_argument);
  EXPECT_THROW(zoom_region(0.5, 0.5, -2, FULL), std::invalid_argument);
}

TEST(Crop, OutputSizeCapsLongestSideKeepingAspect)
{
  const Size z2 = output_size(zoom_region(0.5, 0.5, 2, FULL), 1024);
  EXPECT_EQ(z2.width, 1024);
  EXPECT_EQ(z2.height, 576);
  const Size z8 = output_size(zoom_region(0.5, 0.5, 8, FULL), 1024);
  EXPECT_EQ(z8.width, 576);
  EXPECT_EQ(z8.height, 324);
  const Size small = output_size(zoom_region(0.5, 0.5, 2, FULL), 256);
  EXPECT_EQ(small.width, 256);
  EXPECT_EQ(small.height, 144);
}

TEST(Crop, OutputSizeIsEvenAndNeverLargerThanRegion)
{
  const Rect region{0, 0, 1002, 750};
  for (const int max_px : {2, 100, 333, 1000, 5000}) {
    const Size s = output_size(region, max_px);
    EXPECT_EQ(s.width % 2, 0);
    EXPECT_EQ(s.height % 2, 0);
    EXPECT_LE(s.width, region.width);
    EXPECT_LE(s.height, region.height);
    EXPECT_LE(s.width, max_px < 2 ? 2 : max_px);
  }
}

TEST(Crop, AfWindowAtCentre) { expect_rect(af_window(0.5, 0.5, 0.25, Rect{0, 0, 4000, 2000}), 1500, 750, 1000, 500); }

TEST(Crop, AfWindowCornersAreClamped)
{
  const Rect max{0, 0, 4000, 2000};
  expect_rect(af_window(0.0, 0.0, 0.25, max), 0, 0, 1000, 500);
  expect_rect(af_window(1.0, 0.0, 0.25, max), 3000, 0, 1000, 500);
  expect_rect(af_window(0.0, 1.0, 0.25, max), 0, 1500, 1000, 500);
  expect_rect(af_window(1.0, 1.0, 0.25, max), 3000, 1500, 1000, 500);
}

TEST(Crop, AfWindowHonoursScalerCropOffset)
{
  const Rect max{16, 32, 4000, 2000};
  expect_rect(af_window(0.5, 0.5, 0.25, max), 1516, 782, 1000, 500);
  expect_rect(af_window(1.0, 1.0, 0.25, max), 3016, 1532, 1000, 500);
  expect_rect(af_window(0.0, 0.0, 0.25, max), 16, 32, 1000, 500);
}

TEST(Crop, AfWindowRejectsInvalidArguments)
{
  const Rect max{0, 0, 4000, 2000};
  EXPECT_THROW(af_window(-0.1, 0.5, 0.25, max), std::invalid_argument);
  EXPECT_THROW(af_window(0.5, 1.1, 0.25, max), std::invalid_argument);
  EXPECT_THROW(af_window(0.5, 0.5, 0.0, max), std::invalid_argument);
  EXPECT_THROW(af_window(0.5, 0.5, 1.5, max), std::invalid_argument);
}
