#include "fixtures.hpp"

#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

namespace fixtures
{

std::string dir() { return STEREO_DEPTH_FIXTURES_DIR; }

std::string path(const std::string & relative) { return dir() + "/" + relative; }

cv::Mat png(const std::string & relative)
{
  cv::Mat image = cv::imread(path(relative), cv::IMREAD_UNCHANGED);
  EXPECT_FALSE(image.empty()) << "cannot read fixture " << relative << "; run test/gen_fixtures.py";
  return image;
}

YAML::Node yaml(const std::string & relative) { return YAML::LoadFile(path(relative)); }

TempDir::TempDir()
{
  std::string pattern = (std::filesystem::temp_directory_path() / "stereo_depth_cpp_XXXXXX").string();
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  const char * made = mkdtemp(buffer.data());
  if (made == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  path_ = made;
}

TempDir::~TempDir()
{
  std::error_code ignored;
  std::filesystem::remove_all(path_, ignored);
}

std::string TempDir::write(const std::string & name, const std::string & text) const
{
  const std::string full = path_ + "/" + name;
  std::ofstream(full) << text;
  return full;
}

void copy_calibration(const std::string & case_name, const std::string & directory)
{
  for (const char * name : {"stereo_left.yaml", "stereo_right.yaml", "stereo.yaml"}) {
    std::filesystem::copy_file(
      path(case_name + "/calibration/" + name), directory + "/" + name,
      std::filesystem::copy_options::overwrite_existing);
  }
}

ImageBuffer image_buffer(const cv::Mat & grey, const std::string & encoding, std::int64_t stamp_ns)
{
  int channels = 3;
  if (encoding == "mono8") {
    channels = 1;
  } else if (encoding == "bgra8" || encoding == "rgba8") {
    channels = 4;
  }
  ImageBuffer out;
  out.bytes.reserve(static_cast<std::size_t>(grey.total()) * channels);
  for (int y = 0; y < grey.rows; ++y) {
    for (int x = 0; x < grey.cols; ++x) {
      const std::uint8_t v = grey.at<std::uint8_t>(y, x);
      for (int c = 0; c < channels; ++c) {
        out.bytes.push_back(c == 3 ? 255 : v);
      }
    }
  }
  out.view.encoding = encoding;
  out.view.width = static_cast<std::uint32_t>(grey.cols);
  out.view.height = static_cast<std::uint32_t>(grey.rows);
  out.view.step = static_cast<std::uint32_t>(grey.cols * channels);
  out.view.data = out.bytes.data();
  out.view.size = out.bytes.size();
  out.view.stamp_ns = stamp_ns;
  return out;
}

}  // namespace fixtures
