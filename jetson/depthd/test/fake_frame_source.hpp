// Test support shared by test_pipeline.cpp and test_daemon.cpp: a FrameSource the test drives by hand, synthetic
// stereo pairs of a fronto-parallel textured plane at a known disparity, and a ZeroMQ SUB that decodes depth_wire.
#ifndef DEPTHD_TEST_FAKE_FRAME_SOURCE_HPP_
#define DEPTHD_TEST_FAKE_FRAME_SOURCE_HPP_

#include <zmq.h>

#include <chrono>
#include <ctime>
#include <memory>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>
#include <vector>

#include "depthd/calibration.hpp"
#include "depthd/pipeline.hpp"
#include "ros2_common_cpp/depth_wire.hpp"

namespace depthd_test
{
namespace wire = ros2_common_cpp::depth_wire;

inline int64_t realtime_ns()
{
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

// A FrameSource whose frames come from the test: push_pair / push_rgb run the pipeline's callbacks on the calling
// thread, exactly like the capture's streaming threads do.
class FakeFrameSource : public depthd::FrameSource
{
public:
  void set_pair_callback(PairCallback cb) override { pair_cb_ = std::move(cb); }
  void set_rgb_callback(RgbCallback cb) override { rgb_cb_ = std::move(cb); }
  bool start(std::string * error) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++start_calls_;
    if (!start_error_.empty()) {
      if (error) *error = start_error_;
      return false;
    }
    running_ = true;
    return true;
  }
  void stop() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  depthd::CaptureStatus status() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }
  depthd::PairingStats pairing_stats() const override { return {}; }
  void set_jpeg_quality(int quality) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    jpeg_quality_ = quality;
  }

  void set_status(const depthd::CaptureStatus & status)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = status;
  }
  void set_start_error(const std::string & error)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    start_error_ = error;
  }
  int start_calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return start_calls_;
  }
  int jpeg_quality() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return jpeg_quality_;
  }
  void push_pair(const depthd::StereoPair & pair)
  {
    if (pair_cb_) pair_cb_(pair);
  }
  void push_rgb(const depthd::RgbJpeg & jpeg)
  {
    if (rgb_cb_) rgb_cb_(jpeg);
  }

private:
  mutable std::mutex mutex_;
  PairCallback pair_cb_;
  RgbCallback rgb_cb_;
  depthd::CaptureStatus status_{depthd::CaptureState::ok, "", true, false, false};
  std::string start_error_;
  int start_calls_ = 0;
  int jpeg_quality_ = 0;
  bool running_ = false;
};

// Raw (unrectified) images of a textured fronto-parallel plane seen at `disparity` pixels in the rectified frame of
// `cal`. Each raw pixel samples the rectified scene at its undistorted, rectified position, so rectifying the pair
// with `cal` gives back a pure horizontal shift: rectified_right(x - d, y) = rectified_left(x, y).
struct RawPair
{
  cv::Mat left;
  cv::Mat right;
};

inline RawPair make_raw_pair(const depthd::StereoCalibration & cal, double disparity, unsigned seed = 7)
{
  const int w = cal.size.width;
  const int h = cal.size.height;
  const int margin = 64;
  cv::Mat texture(h + 2 * margin, w + static_cast<int>(disparity) + 2 * margin, CV_8UC1);
  cv::RNG rng(seed);
  rng.fill(texture, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(texture, texture, cv::Size(3, 3), 0.8);

  auto render = [&](const cv::Mat & K, const cv::Mat & D, const cv::Mat & R, const cv::Mat & P, double shift) {
    std::vector<cv::Point2f> raw;
    raw.reserve(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) raw.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }
    std::vector<cv::Point2f> rect;
    cv::undistortPoints(raw, rect, K, D, R, P);
    cv::Mat map_x(h, w, CV_32FC1);
    cv::Mat map_y(h, w, CV_32FC1);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const cv::Point2f & p = rect[static_cast<size_t>(y) * w + x];
        map_x.at<float>(y, x) = static_cast<float>(p.x + shift + margin);
        map_y.at<float>(y, x) = p.y + margin;
      }
    }
    cv::Mat out;
    cv::remap(texture, out, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_REFLECT);
    return out;
  };
  RawPair pair;
  pair.left = render(cal.K1, cal.D1, cal.R1, cal.P1, 0.0);
  pair.right = render(cal.K2, cal.D2, cal.R2, cal.P2, disparity);
  return pair;
}

inline depthd::StereoPair upload_pair(const RawPair & raw, int64_t capture_realtime_ns, uint64_t sequence)
{
  depthd::StereoPair pair;
  pair.left.upload(raw.left);
  pair.right.upload(raw.right);
  pair.capture_realtime_ns = capture_realtime_ns;
  pair.right_minus_left_ns = 1500000;
  pair.left_sequence = sequence;
  pair.right_sequence = sequence + 1000;
  return pair;
}

// A real JPEG (encoded on the CPU, the test has no nvjpegenc) kept alive by the RgbJpeg hold.
inline depthd::RgbJpeg make_jpeg(int width, int height, int64_t capture_realtime_ns, uint64_t sequence)
{
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(30, 120, 200));
  auto bytes = std::make_shared<std::vector<uint8_t>>();
  cv::imencode(".jpg", img, *bytes);
  depthd::RgbJpeg jpeg;
  jpeg.data = bytes->data();
  jpeg.size = bytes->size();
  jpeg.capture_realtime_ns = capture_realtime_ns;
  jpeg.sequence = sequence;
  jpeg.hold = bytes;
  return jpeg;
}

// ZeroMQ SUB on loopback that decodes every message with depth_wire.
class Sub
{
public:
  explicit Sub(int port, int timeout_ms = 200)
  {
    ctx_ = zmq_ctx_new();
    sock_ = zmq_socket(ctx_, ZMQ_SUB);
    zmq_setsockopt(sock_, ZMQ_SUBSCRIBE, "", 0);
    zmq_setsockopt(sock_, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
    const std::string ep = "tcp://127.0.0.1:" + std::to_string(port);
    zmq_connect(sock_, ep.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let the subscription reach the PUB
  }
  ~Sub()
  {
    int linger = 0;
    zmq_setsockopt(sock_, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_close(sock_);
    zmq_ctx_term(ctx_);
  }
  Sub(const Sub &) = delete;
  Sub & operator=(const Sub &) = delete;

  // One decoded message, or false when none arrives within the timeout.
  bool recv(wire::Decoded & out)
  {
    std::vector<std::vector<uint8_t>> parts;
    for (;;) {
      zmq_msg_t m;
      zmq_msg_init(&m);
      if (zmq_msg_recv(&m, sock_, 0) < 0) {
        zmq_msg_close(&m);
        return false;
      }
      const auto * p = static_cast<const uint8_t *>(zmq_msg_data(&m));
      parts.emplace_back(p, p + zmq_msg_size(&m));
      const bool more = zmq_msg_more(&m) != 0;
      zmq_msg_close(&m);
      if (!more) break;
    }
    if (parts.size() != 3) return false;
    out = wire::decode(
      std::string(parts[0].begin(), parts[0].end()), parts[1].data(), parts[1].size(), parts[2].data(),
      parts[2].size());
    return true;
  }

  // Every message that arrives within `window`.
  std::vector<wire::Decoded> drain(std::chrono::milliseconds window)
  {
    std::vector<wire::Decoded> out;
    const auto end = std::chrono::steady_clock::now() + window;
    while (std::chrono::steady_clock::now() < end) {
      wire::Decoded d;
      if (recv(d)) out.push_back(std::move(d));
    }
    return out;
  }

private:
  void * ctx_ = nullptr;
  void * sock_ = nullptr;
};

}  // namespace depthd_test

#endif  // DEPTHD_TEST_FAKE_FRAME_SOURCE_HPP_
