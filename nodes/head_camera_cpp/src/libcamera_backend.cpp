// LibcameraBackend: the only code that touches a camera (design D7). Mode switch = stop -> configure -> start, because
// Camera::configure() is only allowed while Acquired or Configured (libcamera camera.cpp:1195). Buffer allocation,
// mmap and request reuse follow camera_ros CameraNode.cpp (pin 8f792e27) and rpicam-apps, with any number of buffers
// per request set. Version note: see libcamera_backend.hpp (libcamera 0.7.2 in docker and in the Pi fork, no guards).

#include "head_camera_cpp/libcamera_backend.hpp"

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <sys/mman.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <exception>
#include <iostream>
#include <numeric>
#include <sstream>
#include <utility>

namespace head_camera_cpp
{

namespace
{

/// Buffers of the video stream (libcamera's default for the VideoRecording role).
constexpr unsigned int VIDEO_BUFFER_COUNT = 4;
/// Planes of a planar YUV 4:2:0 frame.
constexpr std::size_t YUV420_PLANES = 3;

/// @return The text of a negative libcamera return code ("-16 (Device or resource busy)").
std::string error_text(int ret)
{
  std::ostringstream text;
  text << ret << " (" << std::strerror(-ret) << ")";
  return text.str();
}

/// @return The mode's name for log lines.
std::string mode_name(Mode mode) { return mode == Mode::Video ? "video" : "still"; }

/// Metadata of one completed request.
struct FrameInfo
{
  int64_t stamp_ns;
  std::optional<AfState> af_state;
  std::optional<double> lens_position;
};

/// @return SensorTimestamp (else the buffer timestamp), AfState and LensPosition of a completed request.
FrameInfo frame_info(const libcamera::Request & request, const libcamera::FrameBuffer & buffer)
{
  const libcamera::ControlList & metadata = request.metadata();
  FrameInfo info{static_cast<int64_t>(buffer.metadata().timestamp), std::nullopt, std::nullopt};
  if (const auto stamp = metadata.get(libcamera::controls::SensorTimestamp)) {
    info.stamp_ns = *stamp;
  }
  if (const auto state = metadata.get(libcamera::controls::AfState)) {
    info.af_state = to_af_state(*state);
  }
  if (const auto lens = metadata.get(libcamera::controls::LensPosition)) {
    info.lens_position = static_cast<double>(*lens);
  }
  return info;
}

}  // namespace

LibcameraBackend::LibcameraBackend(const Config & config, LogFunction log)
: config_(config), log_(std::move(log)), pending_controls_(libcamera::controls::controls)
{
  dispatcher_ = std::thread([this]() { dispatch_video(); });
  ensure_open();
}

LibcameraBackend::~LibcameraBackend()
{
  stop();
  release_buffers();
  close_camera();
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
  }
  frame_ready_.notify_all();
  dispatcher_.join();
  // manager_ is destroyed (and stopped) last, after the camera it handed out is released.
}

void LibcameraBackend::log(LogLevel level, const std::string & message) const
{
  if (log_) {
    log_(level, message);
    return;
  }
  static const char * const NAMES[] = {"INFO", "WARN", "ERROR"};
  std::cerr << "[head_camera libcamera " << NAMES[static_cast<int>(level)] << "] " << message << std::endl;
}

bool LibcameraBackend::ensure_open()
{
  if (camera_) {
    return true;
  }
  if (!manager_) {
    auto manager = std::make_unique<libcamera::CameraManager>();
    const int ret = manager->start();
    if (ret < 0) {
      log(LogLevel::Error, "CameraManager start failed: " + error_text(ret));
      return false;
    }
    manager_ = std::move(manager);
  }
  std::shared_ptr<libcamera::Camera> camera = manager_->get(config_.camera_id);
  if (!camera) {
    std::string available;
    for (const auto & other : manager_->cameras()) {
      available += (available.empty() ? "" : ", ") + other->id();
    }
    log(
      LogLevel::Error,
      "camera '" + config_.camera_id + "' not found; available: " + (available.empty() ? "none" : available));
    return false;
  }
  const int ret = camera->acquire();
  if (ret < 0) {
    log(LogLevel::Error, "acquiring camera '" + config_.camera_id + "' failed: " + error_text(ret));
    return false;
  }
  camera->requestCompleted.connect(this, &LibcameraBackend::request_completed);
  camera_ = std::move(camera);
  log(LogLevel::Info, "acquired camera '" + config_.camera_id + "'");
  return true;
}

void LibcameraBackend::close_camera()
{
  if (!camera_) {
    return;
  }
  camera_->requestCompleted.disconnect(this);
  const int ret = camera_->release();
  if (ret < 0) {
    log(LogLevel::Error, "releasing the camera failed: " + error_text(ret));
  }
  camera_.reset();
}

void LibcameraBackend::release_buffers()
{
  // Requests reference the buffers, the buffers back the mappings: drop them in that order, while stopped.
  requests_.clear();
  planes_.clear();
  for (const Mapping & mapping : mappings_) {
    if (munmap(mapping.address, mapping.length) != 0) {
      log(LogLevel::Error, std::string("munmap failed: ") + std::strerror(errno));
    }
  }
  mappings_.clear();
  if (allocator_ && stream_) {
    const int ret = allocator_->free(stream_);
    if (ret < 0) {
      log(LogLevel::Error, "freeing the frame buffers failed: " + error_text(ret));
    }
  }
  allocator_.reset();
  configuration_.reset();
  stream_ = nullptr;
  mode_.reset();
  frame_size_ = Size{0, 0};
  stride_ = 0;
}

bool LibcameraBackend::map_buffers()
{
  for (const std::unique_ptr<libcamera::FrameBuffer> & buffer : allocator_->buffers(stream_)) {
    // Planes usually share one dmabuf at different offsets: map each dmabuf once, over every plane it holds.
    std::map<int, std::size_t> extent;
    for (const libcamera::FrameBuffer::Plane & plane : buffer->planes()) {
      const std::size_t end = static_cast<std::size_t>(plane.offset) + plane.length;
      extent[plane.fd.get()] = std::max(extent[plane.fd.get()], end);
    }
    std::map<int, const uint8_t *> base;
    for (const auto & [fd, length] : extent) {
      void * address = mmap(nullptr, length, PROT_READ, MAP_SHARED, fd, 0);
      if (address == MAP_FAILED) {
        log(LogLevel::Error, std::string("mmap of a frame buffer failed: ") + std::strerror(errno));
        return false;
      }
      mappings_.push_back(Mapping{address, length});
      base[fd] = static_cast<const uint8_t *>(address);
    }
    BufferPlanes & planes = planes_[buffer.get()];
    for (const libcamera::FrameBuffer::Plane & plane : buffer->planes()) {
      planes.data.push_back(base[plane.fd.get()] + plane.offset);
      planes.length.push_back(plane.length);
    }
  }
  return true;
}

bool LibcameraBackend::configure(Mode mode)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      log(LogLevel::Error, "configure(" + mode_name(mode) + ") while the stream runs; stop it first");
      return false;
    }
  }
  release_buffers();
  if (!ensure_open()) {
    return false;
  }

  const bool video = mode == Mode::Video;
  // The output stream plus a Raw stream whose size selects the sensor mode (2304:1296 binned or 4608:2592 full).
  std::unique_ptr<libcamera::CameraConfiguration> configuration = camera_->generateConfiguration(
    {video ? libcamera::StreamRole::VideoRecording : libcamera::StreamRole::StillCapture, libcamera::StreamRole::Raw});
  if (!configuration || configuration->size() != 2) {
    log(LogLevel::Error, "the camera cannot generate a " + mode_name(mode) + " + raw configuration");
    return false;
  }
  const libcamera::PixelFormat format = video ? libcamera::formats::RGB888 : libcamera::formats::YUV420;
  libcamera::StreamConfiguration & output = configuration->at(0);
  output.pixelFormat = format;
  if (video) {
    output.size = libcamera::Size(config_.width, config_.height);
    output.bufferCount = VIDEO_BUFFER_COUNT;
  } else {
    output.size = libcamera::Size(config_.zoom.still_size[0], config_.zoom.still_size[1]);
    output.bufferCount = static_cast<unsigned int>(config_.zoom.still_buffers);
  }
  const std::array<int, 2> & sensor = video ? config_.sensor_mode : config_.zoom.still_size;
  configuration->at(1).size = libcamera::Size(sensor[0], sensor[1]);

  const libcamera::CameraConfiguration::Status status = configuration->validate();
  if (status == libcamera::CameraConfiguration::Invalid) {
    log(LogLevel::Error, "the " + mode_name(mode) + " configuration is invalid");
    return false;
  }
  if (status == libcamera::CameraConfiguration::Adjusted) {
    log(
      LogLevel::Warn, "the " + mode_name(mode) + " configuration was adjusted to " + output.toString() + " + raw " +
                        configuration->at(1).toString());
  }
  if (output.pixelFormat != format) {
    log(LogLevel::Error, "the camera cannot deliver " + format.toString() + ", only " + output.pixelFormat.toString());
    return false;
  }
  int ret = camera_->configure(configuration.get());
  if (ret < 0) {
    log(LogLevel::Error, "configuring the " + mode_name(mode) + " mode failed: " + error_text(ret));
    return false;
  }

  stream_ = output.stream();
  allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
  ret = allocator_->allocate(stream_);
  if (ret <= 0) {
    log(
      LogLevel::Error, "allocating the " + mode_name(mode) + " buffers failed: " + error_text(ret < 0 ? ret : -ENOMEM));
    release_buffers();
    return false;
  }
  frame_size_ = Size{static_cast<int>(output.size.width), static_cast<int>(output.size.height)};
  stride_ = static_cast<int>(output.stride);
  if (!map_buffers()) {
    release_buffers();
    return false;
  }

  // Every buffer must hold a whole frame: one packed BGR plane, or Y, U and V (in 3 planes or 1).
  const auto rows = static_cast<std::size_t>(frame_size_.height);
  const auto stride = static_cast<std::size_t>(stride_);
  const std::size_t needed = video ? stride * rows : stride * rows + (stride / 2) * (rows / 2) * 2;
  for (const auto & entry : planes_) {
    const BufferPlanes & planes = entry.second;
    const std::size_t total = std::accumulate(planes.length.begin(), planes.length.end(), std::size_t{0});
    const bool layout_ok =
      video ? !planes.data.empty() : (planes.data.size() == 1 || planes.data.size() == YUV420_PLANES);
    if (!layout_ok || total < needed) {
      log(
        LogLevel::Error, "unexpected " + mode_name(mode) + " buffer layout: " + std::to_string(planes.data.size()) +
                           " planes, " + std::to_string(total) + " bytes, " + std::to_string(needed) + " needed");
      release_buffers();
      return false;
    }
  }

  for (const std::unique_ptr<libcamera::FrameBuffer> & buffer : allocator_->buffers(stream_)) {
    std::unique_ptr<libcamera::Request> request = camera_->createRequest();
    if (!request) {
      log(LogLevel::Error, "creating a request failed");
      release_buffers();
      return false;
    }
    ret = request->addBuffer(stream_, buffer.get());
    if (ret < 0) {
      log(LogLevel::Error, "adding a buffer to a request failed: " + error_text(ret));
      release_buffers();
      return false;
    }
    requests_.push_back(std::move(request));
  }
  configuration_ = std::move(configuration);
  mode_ = mode;
  log(
    LogLevel::Info, "configured the " + mode_name(mode) + " mode: " + output.toString() + " + raw " +
                      configuration_->at(1).toString() + ", " + std::to_string(requests_.size()) + " buffers");
  return true;
}

bool LibcameraBackend::start_stream(const libcamera::ControlList & controls)
{
  const int ret = camera_->start(&controls);
  if (ret < 0) {
    log(LogLevel::Error, "starting the " + mode_name(*mode_) + " stream failed: " + error_text(ret));
    return false;
  }
  bool queued = true;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    for (const std::unique_ptr<libcamera::Request> & request : requests_) {
      request->reuse(libcamera::Request::ReuseBuffers);
      const int queue_ret = camera_->queueRequest(request.get());
      if (queue_ret < 0) {
        log(LogLevel::Error, "queueing a request failed: " + error_text(queue_ret));
        queued = false;
        break;
      }
    }
  }
  if (!queued) {
    stop();
    return false;
  }
  return true;
}

bool LibcameraBackend::start_video()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      if (mode_ == Mode::Video) {
        return true;
      }
      log(LogLevel::Error, "start_video while the still stream runs; stop it first");
      return false;
    }
  }
  if (mode_ != Mode::Video && !configure(Mode::Video)) {
    return false;
  }
  // AfMetering Auto on top of the D2 video controls: a zoom's windowed AF must not keep metering its window once the
  // whole-frame continuous AF resumes (libcamera keeps AF settings across requests).
  bool focus_locked = false;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    focus_locked = focus_locked_;
  }
  libcamera::ControlList controls = video_controls(config_, focus_locked);
  controls.set(libcamera::controls::AfMetering, libcamera::controls::AfMeteringAuto);
  return start_stream(controls);
}

bool LibcameraBackend::start_still()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      log(LogLevel::Error, "start_still while a stream runs; stop it first");
      return false;
    }
  }
  if (mode_ != Mode::Still) {
    log(LogLevel::Error, "start_still without the still configuration; configure(Mode::Still) first");
    return false;
  }
  return start_stream(still_controls());
}

void LibcameraBackend::stop()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return;
    }
    // From here no completed request is handed off or re-queued.
    running_ = false;
  }
  // Not under mutex_: stop() cancels the queued requests, whose requestCompleted runs on libcamera's thread meanwhile.
  const int ret = camera_->stop();
  if (ret < 0) {
    log(LogLevel::Error, "stopping the camera failed: " + error_text(ret));
  }
  std::unique_lock<std::mutex> lock(mutex_);
  // A video callback in progress still reads its buffer: wait until it returns (unless this is that callback).
  if (std::this_thread::get_id() != dispatcher_.get_id()) {
    frame_ready_.wait(lock, [this]() { return !video_busy_; });
  }
  video_slot_ = nullptr;
  still_queue_.clear();
  still_held_ = nullptr;
  af_marker_ = nullptr;
  af_trigger_pending_ = false;
  pending_controls_.clear();
}

void LibcameraBackend::requeue_locked(libcamera::Request * request)
{
  request->reuse(libcamera::Request::ReuseBuffers);
  if (!pending_controls_.empty()) {
    request->controls().merge(pending_controls_, libcamera::ControlList::MergePolicy::OverwriteExisting);
    pending_controls_.clear();
    if (af_trigger_pending_) {
      af_marker_ = request;
      af_trigger_pending_ = false;
    }
  }
  const int ret = camera_->queueRequest(request);
  if (ret < 0) {
    log(LogLevel::Error, "re-queueing a request failed: " + error_text(ret));
  }
}

void LibcameraBackend::request_completed(libcamera::Request * request)
{
  // libcamera's thread: only hand the frame off. Nothing here waits, and no exception may escape into libcamera.
  try {
    if (request->status() == libcamera::Request::RequestCancelled) {
      return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return;  // completed during stop(): never handed off
    }
    const libcamera::FrameBuffer * buffer = request->findBuffer(stream_);
    if (buffer == nullptr) {
      return;
    }
    const FrameInfo info = frame_info(*request, *buffer);
    // After a trigger, states from before it are stale: keep Scanning until the request carrying it completes.
    const bool fresh = !af_trigger_pending_ && (af_marker_ == nullptr || af_marker_ == request);
    if (af_marker_ == request) {
      af_marker_ = nullptr;
    }
    if (fresh && info.af_state) {
      af_state_ = *info.af_state;
    }
    if (buffer->metadata().status != libcamera::FrameMetadata::FrameSuccess) {
      requeue_locked(request);
      return;
    }
    if (mode_ == Mode::Video) {
      if (video_slot_ != nullptr) {
        requeue_locked(video_slot_);  // the dispatcher never took it: drop the older frame
      }
      video_slot_ = request;
    } else {
      still_queue_.push_back(request);
    }
    frame_ready_.notify_all();
  } catch (const std::exception & e) {
    log(LogLevel::Error, std::string("request completion failed: ") + e.what());
  } catch (...) {
    log(LogLevel::Error, "request completion failed");
  }
}

void LibcameraBackend::dispatch_video()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    frame_ready_.wait(lock, [this]() { return shutdown_ || video_slot_ != nullptr; });
    if (shutdown_) {
      return;
    }
    if (!running_) {
      video_slot_ = nullptr;  // stop() is running: a frame from before it is never delivered
      continue;
    }
    libcamera::Request * request = video_slot_;
    video_slot_ = nullptr;
    video_busy_ = true;
    const std::function<void(const VideoFrame &)> callback = video_callback_;
    const libcamera::FrameBuffer * buffer = request->findBuffer(stream_);
    const VideoFrame frame{
      planes_.at(buffer).data.front(), stride_, frame_size_, frame_info(*request, *buffer).stamp_ns};
    lock.unlock();
    if (callback) {
      try {
        callback(frame);
      } catch (const std::exception & e) {
        log(LogLevel::Error, std::string("video callback failed: ") + e.what());
      } catch (...) {
        log(LogLevel::Error, "video callback failed");
      }
    }
    lock.lock();
    video_busy_ = false;
    // running_ was re-read under the lock: stop() may have cleared it while the callback ran.
    // cppcheck-suppress knownConditionTrueFalse
    if (running_) {
      requeue_locked(request);
    }
    frame_ready_.notify_all();
  }
}

bool LibcameraBackend::set_af(const AfRequest & request)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!running_) {
    log(LogLevel::Error, "set_af while no stream runs");
    return false;
  }
  pending_controls_.merge(af_controls(request), libcamera::ControlList::MergePolicy::OverwriteExisting);
  if (request.trigger) {
    af_state_ = AfState::Scanning;
    af_trigger_pending_ = true;
    af_marker_ = nullptr;
  }
  return true;
}

bool LibcameraBackend::set_focus_lock(bool locked)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  focus_locked_ = locked;
  if (running_ && mode_ == Mode::Video) {
    pending_controls_.merge(focus_controls(config_, locked), libcamera::ControlList::MergePolicy::OverwriteExisting);
  }
  return true;
}

AfState LibcameraBackend::af_state()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return af_state_;
}

std::optional<StillFrame> LibcameraBackend::still_frame(const libcamera::Request & request) const
{
  const libcamera::FrameBuffer * buffer = request.findBuffer(stream_);
  if (buffer == nullptr) {
    log(LogLevel::Error, "a still request carries no buffer");
    return std::nullopt;
  }
  const BufferPlanes & planes = planes_.at(buffer);
  Yuv420View view{};
  view.y_stride = stride_;
  view.uv_stride = stride_ / 2;
  view.size = frame_size_;
  view.y = planes.data[0];
  if (planes.data.size() == YUV420_PLANES) {
    view.u = planes.data[1];
    view.v = planes.data[2];
  } else {
    view.u = view.y + static_cast<std::size_t>(view.y_stride) * static_cast<std::size_t>(frame_size_.height);
    view.v = view.u + static_cast<std::size_t>(view.uv_stride) * static_cast<std::size_t>(frame_size_.height / 2);
  }
  const FrameInfo info = frame_info(request, *buffer);
  return StillFrame{view, info.stamp_ns, info.af_state.value_or(AfState::Idle), info.lens_position};
}

std::optional<StillFrame> LibcameraBackend::wait_still_frame(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!running_ || mode_ != Mode::Still) {
    log(LogLevel::Error, "wait_still_frame while the still stream does not run");
    return std::nullopt;
  }
  // The frame returned last time is no longer valid: give its buffer back to the camera.
  if (still_held_ != nullptr) {
    requeue_locked(still_held_);
    still_held_ = nullptr;
  }
  if (!frame_ready_.wait_for(lock, timeout, [this]() { return !still_queue_.empty() || !running_; })) {
    return std::nullopt;
  }
  if (still_queue_.empty()) {
    return std::nullopt;
  }
  still_held_ = still_queue_.front();
  still_queue_.pop_front();
  return still_frame(*still_held_);
}

void LibcameraBackend::set_video_callback(std::function<void(const VideoFrame &)> callback)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  video_callback_ = std::move(callback);
}

bool LibcameraBackend::reacquire()
{
  log(LogLevel::Warn, "re-acquiring camera '" + config_.camera_id + "'");
  stop();
  release_buffers();
  close_camera();
  return ensure_open();
}

Rect LibcameraBackend::scaler_crop_maximum()
{
  if (!camera_) {
    log(LogLevel::Error, "scaler_crop_maximum without an acquired camera");
    return Rect{0, 0, 0, 0};
  }
  const std::optional<libcamera::Rectangle> crop = camera_->properties().get(libcamera::properties::ScalerCropMaximum);
  if (!crop) {
    log(LogLevel::Error, "the camera reports no ScalerCropMaximum");
    return Rect{0, 0, 0, 0};
  }
  return from_rectangle(*crop);
}

}  // namespace head_camera_cpp
