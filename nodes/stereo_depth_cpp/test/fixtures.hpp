// Test helpers: the golden fixtures written by test/gen_fixtures.py (from the Python node's functions), scratch
// directories and camera Image buffers.
#ifndef FIXTURES_HPP_
#define FIXTURES_HPP_

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "stereo_depth_cpp/pipeline.hpp"

namespace fixtures
{

// test/fixtures (compile definition STEREO_DEPTH_FIXTURES_DIR).
std::string dir();
// dir() + "/" + relative.
std::string path(const std::string & relative);
// A fixture PNG, unchanged depth (CV_8U or CV_16U); fails the test when unreadable.
cv::Mat png(const std::string & relative);
// A fixture YAML file.
YAML::Node yaml(const std::string & relative);

// A fresh, empty directory under the system temp dir, removed by the destructor.
class TempDir
{
public:
  TempDir();
  ~TempDir();
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;
  const std::string & path() const { return path_; }
  // Write text to path()/name and return the full path.
  std::string write(const std::string & name, const std::string & text) const;

private:
  std::string path_;
};

// Copy the three calibration files of a fixture case (e.g. "ideal") into directory.
void copy_calibration(const std::string & case_name, const std::string & directory);

// A packed camera Image buffer of a grey image: replicated to 3 channels for bgr8/rgb8 (as conftest.image_msg does),
// 4 channels with alpha 255 for bgra8/rgba8, unchanged for mono8.
struct ImageBuffer
{
  std::vector<std::uint8_t> bytes;
  stereo_depth_cpp::ImageView view;
};
ImageBuffer image_buffer(const cv::Mat & grey, const std::string & encoding, std::int64_t stamp_ns);

}  // namespace fixtures

#endif  // FIXTURES_HPP_
