#pragma once

// Scripted CameraBackend and Clock for tests (not a test itself: the gtest glob matches only test_*.cpp). Reused by
// the zoom controller tests and the node tests.
//
// FakeClock only moves when someone sleeps on it or advances it. FakeCameraBackend logs every call by name
// ("start_video", "stop", "configure(Video)", "configure(Still)", "start_still", "set_af", "af_state",
// "wait_still_frame", "set_video_callback", "reacquire", "scaler_crop_maximum"), answers from a script (AF state
// sequence, per-call failures, per-call delays on the clock, a still frame budget) and serves still frames from a
// synthetic YUV 4:2:0 buffer and video frames from a synthetic BGR buffer.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "head_camera_cpp/camera_backend.hpp"

namespace head_camera_cpp::testing
{

/// A clock that moves only on sleep_for and advance. Thread-safe.
class FakeClock : public Clock
{
public:
  static constexpr std::chrono::seconds START{1000};

  TimePoint now() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return now_;
  }

  void sleep_for(Duration duration) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sleeps_.push_back(duration);
    if (duration > Duration::zero()) {
      now_ += duration;
    }
  }

  /// @param duration Time to move forward (no sleep is logged).
  void advance(Duration duration)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    now_ += duration;
  }

  /// @return Time since construction.
  Duration elapsed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return now_ - TimePoint(START);
  }

  /// @return Every sleep_for duration, in order.
  std::vector<Duration> sleeps()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sleeps_;
  }

private:
  std::mutex mutex_;
  TimePoint now_{START};
  std::vector<Duration> sleeps_;
};

/// A scripted camera. Set the public script fields before use; read the log through calls().
class FakeCameraBackend : public CameraBackend
{
public:
  using Duration = Clock::Duration;

  /**
   * @param still_size Size of the synthetic still frames (even width and height).
   * @param clock When set, still frames and per-call delays advance it.
   * @param video_size Size of the synthetic video frames.
   */
  explicit FakeCameraBackend(
    Size still_size = Size{640, 360}, FakeClock * clock = nullptr, Size video_size = Size{64, 36})
  : still_size_(still_size), video_size_(video_size), clock_(clock)
  {
    const auto luma = static_cast<size_t>(still_size.width) * static_cast<size_t>(still_size.height);
    y_plane_.resize(luma);
    u_plane_.assign(luma / 4, 100);
    v_plane_.assign(luma / 4, 160);
    for (int y = 0; y < still_size.height; ++y) {
      for (int x = 0; x < still_size.width; ++x) {
        y_plane_[static_cast<size_t>(y * still_size.width + x)] = static_cast<uint8_t>((x * 3 + y * 5) % 256);
      }
    }
    bgr_.resize(static_cast<size_t>(video_size.width * video_size.height * 3));
    for (size_t i = 0; i < bgr_.size(); ++i) {
      bgr_[i] = static_cast<uint8_t>((i * 7) % 256);
    }
  }

  // --- script ---------------------------------------------------------------------------------------------------

  /// af_state() answers these in order, then repeats the last one; empty answers Idle.
  std::vector<AfState> af_states;
  /// Still frames wait_still_frame delivers in total; negative is unlimited. Past it waits time out.
  int still_frame_budget = -1;
  /// Time each delivered still frame takes on the clock.
  Duration still_frame_interval = std::chrono::milliseconds(33);
  /// Metadata of every still frame.
  AfState still_af_state = AfState::Focused;
  std::optional<double> still_lens_position = 2.5;
  /// Deliver frames with a null view (an invalid frame for the encoder).
  bool corrupt_still_frames = false;
  /// Returned by scaler_crop_maximum().
  Rect scaler_crop{16, 8, 4576, 2576};
  /// Called after each logged call with its name, outside the lock (e.g. to start a concurrent request).
  std::function<void(const std::string &)> on_call;

  /**
   * Make the next `times` calls named `call` fail (configure(Video), configure(Still), start_video, start_still,
   * set_af, reacquire). Other calls of that name succeed.
   *
   * @param call The call name.
   * @param times How many consecutive calls fail.
   */
  void fail(const std::string & call, int times = 1)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < times; ++i) {
      results_[call].push_back(false);
    }
  }

  /**
   * @param call The call name.
   * @param duration Time every call of that name takes on the clock.
   */
  void delay(const std::string & call, Duration duration)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    delays_[call] = duration;
  }

  /// Put the camera in the stopped, unconfigured state (the default is video configured and streaming, which is
  /// what ZoomController expects; a node test starting the camera itself calls this first).
  void reset_stopped()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    configured_ = false;
  }

  // --- inspection -----------------------------------------------------------------------------------------------

  /// @return Every call name so far, in order.
  std::vector<std::string> calls()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

  /// @return Number of calls named `call` so far.
  int count(const std::string & call)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(std::count(calls_.begin(), calls_.end(), call));
  }

  /// @return Every set_focus_lock argument so far, in order (not in calls(): the restore sequences stay exact).
  std::vector<bool> focus_lock_calls()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return focus_lock_calls_;
  }

  /// @return The focus lock state the camera currently holds (false until set_focus_lock(true)).
  bool focus_locked()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return focus_locked_;
  }

  /// @return The focus lock state at each start_video call, in order.
  std::vector<bool> lock_at_start_video()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return lock_at_start_video_;
  }

  /// @return Every AfRequest set_af received.
  std::vector<AfRequest> af_requests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return af_requests_;
  }

  /// @return Stamps of the still frames delivered, in order.
  std::vector<int64_t> still_stamps()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return still_stamps_;
  }

  /// @return True while the video stream runs (start_video succeeded, no stop since).
  bool video_running()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_ && mode_ == Mode::Video;
  }

  /**
   * Deliver one synthetic video frame to the registered callback, as the camera thread would.
   *
   * @param stamp_ns The frame's capture stamp.
   * @return true when a callback ran (video running and a callback registered).
   */
  bool emit_video_frame(int64_t stamp_ns)
  {
    std::function<void(const VideoFrame &)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || mode_ != Mode::Video || !video_callback_) {
        return false;
      }
      callback = video_callback_;
    }
    callback(VideoFrame{bgr_.data(), video_size_.width * 3, video_size_, stamp_ns});
    return true;
  }

  /// @return The synthetic video frame's pixels (what emit_video_frame delivers).
  const std::vector<uint8_t> & video_pixels() const { return bgr_; }

  /// @return The synthetic still frame's view.
  Yuv420View still_view() const
  {
    return Yuv420View{y_plane_.data(),   u_plane_.data(),       v_plane_.data(),
                      still_size_.width, still_size_.width / 2, still_size_};
  }

  // --- CameraBackend --------------------------------------------------------------------------------------------

  bool start_video() override
  {
    const bool ok = record("start_video");
    bool running = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = ok && configured_ && mode_ == Mode::Video;
      running = running_;
      lock_at_start_video_.push_back(focus_locked_);
    }
    notify("start_video");
    return running;
  }

  void stop() override
  {
    record("stop");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
    }
    notify("stop");
  }

  bool configure(Mode mode) override
  {
    const std::string name = mode == Mode::Video ? "configure(Video)" : "configure(Still)";
    const bool ok = record(name);
    bool configured = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      configured_ = ok && !running_;  // like libcamera: configuring a running camera fails
      mode_ = mode;
      configured = configured_;
    }
    notify(name);
    return configured;
  }

  bool start_still() override
  {
    const bool ok = record("start_still");
    bool running = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = ok && configured_ && mode_ == Mode::Still;
      running = running_;
    }
    notify("start_still");
    return running;
  }

  bool set_focus_lock(bool locked) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    focus_lock_calls_.push_back(locked);
    focus_locked_ = locked;
    return true;
  }

  bool set_af(const AfRequest & request) override
  {
    const bool ok = record("set_af");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      af_requests_.push_back(request);
    }
    notify("set_af");
    return ok;
  }

  AfState af_state() override
  {
    record("af_state");
    AfState state = AfState::Idle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!af_states.empty()) {
        state = af_states[std::min(af_index_, af_states.size() - 1)];
        ++af_index_;
      }
    }
    notify("af_state");
    return state;
  }

  std::optional<StillFrame> wait_still_frame(std::chrono::milliseconds timeout) override
  {
    record("wait_still_frame");
    std::optional<StillFrame> frame;
    Duration took = timeout;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool have = running_ && mode_ == Mode::Still && still_frame_budget != 0;
      if (have && still_frame_interval <= timeout) {
        if (still_frame_budget > 0) {
          --still_frame_budget;
        }
        took = still_frame_interval;
        next_stamp_ns_ += std::chrono::duration_cast<std::chrono::nanoseconds>(still_frame_interval).count();
        still_stamps_.push_back(next_stamp_ns_);
        Yuv420View view = still_view();
        if (corrupt_still_frames) {
          view.y = nullptr;
        }
        frame = StillFrame{view, next_stamp_ns_, still_af_state, still_lens_position};
      }
    }
    if (clock_ != nullptr) {
      clock_->advance(took);
    }
    notify("wait_still_frame");
    return frame;
  }

  void set_video_callback(std::function<void(const VideoFrame &)> callback) override
  {
    record("set_video_callback");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      video_callback_ = std::move(callback);
    }
    notify("set_video_callback");
  }

  bool reacquire() override
  {
    const bool ok = record("reacquire");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
      configured_ = false;
    }
    notify("reacquire");
    return ok;
  }

  Rect scaler_crop_maximum() override
  {
    record("scaler_crop_maximum");
    notify("scaler_crop_maximum");
    return scaler_crop;
  }

private:
  /// Log the call, apply its delay and pop its scripted result.
  bool record(const std::string & name)
  {
    Duration took = Duration::zero();
    bool ok = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      calls_.push_back(name);
      const auto d = delays_.find(name);
      if (d != delays_.end()) {
        took = d->second;
      }
      auto r = results_.find(name);
      if (r != results_.end() && !r->second.empty()) {
        ok = r->second.front();
        r->second.pop_front();
      }
    }
    if (clock_ != nullptr && took > Duration::zero()) {
      clock_->advance(took);
    }
    return ok;
  }

  void notify(const std::string & name)
  {
    if (on_call) {
      on_call(name);
    }
  }

  Size still_size_;
  Size video_size_;
  FakeClock * clock_;
  std::vector<uint8_t> y_plane_;
  std::vector<uint8_t> u_plane_;
  std::vector<uint8_t> v_plane_;
  std::vector<uint8_t> bgr_;

  std::mutex mutex_;
  std::vector<std::string> calls_;
  std::map<std::string, std::deque<bool>> results_;
  std::map<std::string, Duration> delays_;
  std::vector<AfRequest> af_requests_;
  std::vector<bool> focus_lock_calls_;
  std::vector<bool> lock_at_start_video_;
  bool focus_locked_ = false;
  std::vector<int64_t> still_stamps_;
  std::function<void(const VideoFrame &)> video_callback_;
  size_t af_index_ = 0;
  int64_t next_stamp_ns_ = 5'000'000'000;
  Mode mode_ = Mode::Video;
  bool configured_ = true;
  bool running_ = true;
};

}  // namespace head_camera_cpp::testing
