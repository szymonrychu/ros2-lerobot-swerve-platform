#include "depthd/capture.hpp"

#include <cuda.h>
#include <cuda_runtime.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"  // cudaEGL.h (CUDA 10.2) has a stray ';'
#include <cudaEGL.h>
#include <dlfcn.h>
#pragma GCC diagnostic pop
#include <fcntl.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <linux/videodev2.h>
#include <nvbuf_utils.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace depthd
{
namespace
{
struct ModeEntry
{
  SensorMode mode;
  int index;
};

// IMX219 modes reported by nvarguscamerasrc on the Nano (L4T R32.6.1), in Argus index order.
const std::array<ModeEntry, 6> kImx219Modes{{
  {{3264, 2464, 21}, 0},
  {{3264, 1848, 28}, 1},
  {{1920, 1080, 30}, 2},
  {{1640, 1232, 30}, 3},
  {{1280, 720, 60}, 4},
  {{1280, 720, 120}, 5},
}};

// nvarguscamerasrc attaches this struct as qdata "GstBufferMetaData" on every buffer it creates (L4T R32.x
// gstnvarguscamerasrc.hpp AuxBufferData); timestamp is the sensor start-of-frame time, CLOCK_MONOTONIC ns.
struct ArgusAuxData
{
  gint64 frame_num;
  gint64 timestamp;
  void * sensor_data;
};

constexpr std::size_t kTimestampRing = 32;
constexpr std::size_t kMaxPoolFrames = 16;
constexpr int kLockReadbackFrames = 10;  // frames after the lock before the locked values are read back
constexpr int64_t kWatchdogTickNs = 100000000;
constexpr uint64_t kAlignFrames = 5;  // frames per camera before the phase is measured

int64_t clock_ns(clockid_t clock)
{
  timespec ts{};
  clock_gettime(clock, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

const char * side_name(CameraSide side) { return side == CameraSide::left ? "left" : "right"; }

void default_log(const std::string & line) { std::cerr << "depthd capture: " << line << std::endl; }

// Runtime API call that makes the primary CUDA context current on this (GStreamer streaming) thread, so the driver
// API EGL interop calls below have a context.
bool ensure_cuda_context()
{
  thread_local bool ready = false;
  if (!ready) ready = cudaFree(nullptr) == cudaSuccess;
  return ready;
}

// The process-wide default EGL display, initialised once and never terminated: nvarguscamerasrc uses the same display
// for its EGLStream, and terminating it breaks every later camera start in the process. EGL_NO_DISPLAY lets nvbuf_utils
// use its own instance.
EGLDisplay shared_egl_display()
{
  static const EGLDisplay display = [] {
    EGLDisplay d = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    return d != EGL_NO_DISPLAY && eglInitialize(d, nullptr, nullptr) ? d : EGL_NO_DISPLAY;
  }();
  return display;
}

// nvjpegenc calls the libjpeg API, which L4T implements in tegra/libnvjpeg.so with a larger struct
// (jpeg_compress_struct 728 bytes). Any process that also links OpenCV imgcodecs (depthd does, for the PNG parts of
// GET /calibration/capture) has libjpeg.so.8 (584 bytes) in its global scope, so the plugin's jpeg_* calls would bind
// there and libjpeg exits the process: "JPEG parameter struct mismatch: library thinks size is 584, caller expects
// 728". Loading the plugin first with RTLD_DEEPBIND puts its own dependencies (libnvjpeg) ahead of the global scope;
// GStreamer's later g_module_open of the same file gets this already-bound handle. OpenCV keeps libjpeg.so.8.
constexpr const char * kNvJpegPlugin = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvjpeg.so";

void bind_nvjpeg_plugin(const std::function<void(const std::string &)> & log)
{
  static std::once_flag once;
  std::call_once(once, [&log] {
    if (!dlopen(kNvJpegPlugin, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND)) {
      const char * err = dlerror();
      if (log) log(std::string("preloading the nvjpegenc plugin failed: ") + (err ? err : "unknown error"));
    }
  });
}

std::string cu_error(CUresult r)
{
  const char * s = nullptr;
  cuGetErrorString(r, &s);
  return s ? s : ("CUresult " + std::to_string(static_cast<int>(r)));
}

// Fixed set of GPU frame buffers reused across frames (no per-frame cudaMalloc). A buffer is free when only the pool
// holds a reference to it; the pairer and the pipeline hold the others through GpuMat reference counting.
class GpuFramePool
{
public:
  bool acquire(int rows, int cols, cv::cuda::GpuMat & out)
  {
    for (auto & m : frames_) {
      if (m.refcount && *m.refcount == 1 && m.rows == rows && m.cols == cols) {
        out = m;
        return true;
      }
    }
    if (frames_.size() >= kMaxPoolFrames) return false;
    frames_.emplace_back(rows, cols, CV_8UC1);
    out = frames_.back();
    return true;
  }

private:
  std::vector<cv::cuda::GpuMat> frames_;
};
}  // namespace

int imx219_sensor_mode(const SensorMode & mode)
{
  for (const auto & e : kImx219Modes) {
    if (e.mode == mode) return e.index;
  }
  return -1;
}

std::string build_pipeline_description(const Config & config, CameraSide side)
{
  const auto & cap = config.capture;
  const int mode = imx219_sensor_mode(cap.sensor_mode);
  if (mode < 0) {
    std::ostringstream msg;
    msg << "capture.sensor_mode: the IMX219 has no " << cap.sensor_mode.width << "x" << cap.sensor_mode.height << "@"
        << cap.sensor_mode.fps << " mode (supported:";
    for (const auto & e : kImx219Modes) msg << " " << e.mode.width << "x" << e.mode.height << "@" << e.mode.fps;
    msg << ")";
    throw std::invalid_argument(msg.str());
  }
  const bool left = side == CameraSide::left;
  const std::string name = side_name(side);
  const int sensor_id = left ? cap.left_sensor_id : cap.right_sensor_id;

  std::ostringstream s;
  s << "nvarguscamerasrc name=" << name << "_src sensor-id=" << sensor_id << " sensor-mode=" << mode
    << " ee-mode=0 tnr-mode=1 aelock=false awblock=false ! video/x-raw(memory:NVMM),width=" << cap.sensor_mode.width
    << ",height=" << cap.sensor_mode.height << ",format=NV12,framerate=" << cap.sensor_mode.fps << "/1";

  std::ostringstream gray;
  gray << " ! nvvidconv ! " << (cap.zero_copy ? "video/x-raw(memory:NVMM)" : "video/x-raw")
       << ",width=" << config.processing.width << ",height=" << config.processing.height
       << ",format=GRAY8 ! appsink name=" << name << "_sink max-buffers=1 drop=true sync=false";

  if (left && config.rgb.enabled) {
    s << " ! tee name=left_tee left_tee. ! queue max-size-buffers=2 leaky=downstream" << gray.str()
      << " left_tee. ! queue name=rgb_queue max-size-buffers=1 leaky=downstream ! nvvidconv ! "
         "video/x-raw(memory:NVMM),width="
      << config.rgb.width << ",height=" << config.rgb.height
      << ",format=I420 ! nvjpegenc name=rgb_jpeg quality=" << config.rgb.jpeg_quality
      << " ! appsink name=rgb_sink max-buffers=2 drop=true sync=false";
  } else {
    s << gray.str();
  }
  return s.str();
}

std::optional<int64_t> capture_monotonic_ns(
  std::optional<int64_t> sensor_timestamp_ns, uint64_t pts_ns, uint64_t base_time_ns)
{
  if (sensor_timestamp_ns && *sensor_timestamp_ns > 0) return *sensor_timestamp_ns;
  if (pts_ns == GST_CLOCK_TIME_NONE || base_time_ns == GST_CLOCK_TIME_NONE) return std::nullopt;
  return static_cast<int64_t>(pts_ns + base_time_ns);
}

ClockOffset::ClockOffset()
: ClockOffset([] { return clock_ns(CLOCK_MONOTONIC); }, [] { return clock_ns(CLOCK_REALTIME); })
{
}

ClockOffset::ClockOffset(ClockFn monotonic, ClockFn realtime)
: monotonic_(std::move(monotonic)), realtime_(std::move(realtime))
{
  refresh();
}

void ClockOffset::refresh()
{
  std::lock_guard<std::mutex> lock(refresh_mutex_);
  int64_t best_width = std::numeric_limits<int64_t>::max();
  int64_t best_offset = 0;
  int64_t last_mono = 0;
  for (int i = 0; i < 3; ++i) {
    const int64_t m0 = monotonic_();
    const int64_t r = realtime_();
    const int64_t m1 = monotonic_();
    last_mono = m1;
    if (m1 - m0 < best_width) {
      best_width = m1 - m0;
      best_offset = r - (m0 + (m1 - m0) / 2);
    }
  }
  offset_ns_.store(best_offset);
  sampled_at_mono_ns_.store(last_mono);
}

int64_t ClockOffset::to_realtime(int64_t monotonic_ns)
{
  if (monotonic_() - sampled_at_mono_ns_.load() >= kRefreshIntervalNs) refresh();
  return monotonic_ns + offset_ns_.load();
}

bool camera_timed_out(int64_t now_mono_ns, int64_t last_frame_mono_ns, int64_t started_mono_ns)
{
  const int64_t since = last_frame_mono_ns ? last_frame_mono_ns : started_mono_ns;
  return now_mono_ns - since > StereoCapture::kCameraTimeoutNs;
}

const char * to_string(CaptureState state)
{
  switch (state) {
    case CaptureState::starting:
      return "starting";
    case CaptureState::ok:
      return "ok";
    case CaptureState::camera_error:
      return "camera_error";
  }
  return "camera_error";
}

RgbThrottle::RgbThrottle(double rgb_fps, double sensor_fps)
: interval_ns_(static_cast<uint64_t>(1e9 / rgb_fps)), tolerance_ns_(static_cast<uint64_t>(0.5e9 / sensor_fps))
{
}

bool RgbThrottle::admit(uint64_t pts_ns)
{
  if (!due_set_ || pts_ns < last_admitted_ns_) {  // first buffer, or PTS went back: the pipeline restarted
    due_set_ = true;
    next_due_ns_ = pts_ns;
  }
  if (pts_ns + tolerance_ns_ < next_due_ns_) return false;
  next_due_ns_ += interval_ns_;
  if (next_due_ns_ + tolerance_ns_ < pts_ns) next_due_ns_ = pts_ns + interval_ns_;  // resync after a gap
  last_admitted_ns_ = pts_ns;
  return true;
}

void RgbThrottle::reset() { due_set_ = false; }

bool ExposureLockPolicy::on_frame(CameraSide side)
{
  if (!enabled_ || locked_) return false;
  ++(side == CameraSide::left ? left_frames_ : right_frames_);
  const int64_t needed = std::max(settle_frames_, 1);
  if (left_frames_ >= needed && right_frames_ >= needed) {
    locked_ = true;
    return true;
  }
  return false;
}

void ExposureLockPolicy::reset()
{
  left_frames_ = 0;
  right_frames_ = 0;
  locked_ = false;
}

std::optional<SensorExposure> read_sensor_exposure(int sensor_id, std::string * error)
{
  const std::string dev = "/dev/video" + std::to_string(sensor_id);
  auto fail = [&](const std::string & why) -> std::optional<SensorExposure> {
    if (error) *error = dev + ": " + why;
    return std::nullopt;
  };
  const int fd = ::open(dev.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) return fail(std::strerror(errno));
  uint32_t exposure_id = 0;
  uint32_t gain_id = 0;
  v4l2_query_ext_ctrl q{};
  q.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  while (::ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) == 0) {
    if (strcasecmp(q.name, "exposure") == 0) exposure_id = q.id;
    if (strcasecmp(q.name, "gain") == 0) gain_id = q.id;
    q.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  }
  if (!exposure_id || !gain_id) {
    ::close(fd);
    return fail("no exposure/gain controls");
  }
  std::array<v4l2_ext_control, 2> ctrls{};
  ctrls[0].id = exposure_id;
  ctrls[1].id = gain_id;
  v4l2_ext_controls req{};
  req.which = V4L2_CTRL_WHICH_CUR_VAL;
  req.count = ctrls.size();
  req.controls = ctrls.data();
  const int rc = ::ioctl(fd, VIDIOC_G_EXT_CTRLS, &req);
  const int err = errno;
  ::close(fd);
  if (rc != 0) return fail(std::string("VIDIOC_G_EXT_CTRLS: ") + std::strerror(err));
  return SensorExposure{ctrls[0].value64, ctrls[1].value64};
}

// ---------------------------------------------------------------------------------------------------------------------

struct StereoCapture::Impl
{
  struct Side
  {
    Impl * impl = nullptr;
    CameraSide side = CameraSide::left;
    int sensor_id = 0;
    GstElement * pipeline = nullptr;
    GstElement * src = nullptr;
    GstElement * sink = nullptr;
    GstBus * bus = nullptr;

    std::mutex ts_mutex;  // source probe thread writes, sink threads read
    struct Stamp
    {
      uint64_t pts = GST_CLOCK_TIME_NONE;
      int64_t sensor_ns = 0;
      int64_t frame_num = -1;
    };
    std::array<Stamp, kTimestampRing> stamps{};
    std::size_t stamp_next = 0;

    std::atomic<int64_t> last_frame_mono_ns{0};
    std::atomic<int64_t> last_sensor_ns{0};  // newest sensor timestamp seen by the source probe
    std::atomic<uint64_t> source_frames{0};  // buffers seen by the source probe since the last (re)start
    std::atomic<int64_t> started_mono_ns{0};
    uint64_t frames = 0;  // sink thread only
    GpuFramePool pool;    // sink thread only
    cv::cuda::Stream stream;
    bool convert_error_logged = false;  // sink thread only

    // Guarded by Impl::mutex.
    std::string error;
    bool error_needs_restart = false;
    int64_t last_restart_mono_ns = 0;
  };

  Config config;
  LogFn log;
  PairCallback pair_cb;
  RgbCallback rgb_cb;
  StereoPairer pairer;
  ClockOffset clock;
  std::array<Side, 2> sides;  // [0] left, [1] right
  GstElement * rgb_sink = nullptr;
  GstElement * rgb_jpeg = nullptr;
  GstElement * rgb_queue = nullptr;
  EGLDisplay egl_display = EGL_NO_DISPLAY;

  mutable std::mutex mutex;
  ExposureLockPolicy lock_policy;
  bool exposure_locked = false;
  int frames_since_lock = -1;  // -1: no readback pending
  bool any_pair = false;
  bool aligning = true;
  int align_attempts = 0;
  std::string start_error;

  std::atomic<bool> running{false};
  std::thread watchdog;
  RgbThrottle rgb_throttle;  // rgb_queue streaming thread only (reset while the left pipeline is stopped)
  uint64_t rgb_sequence = 0;

  Impl(const Config & c, LogFn l)
  : config(c),
    log(l ? std::move(l) : LogFn(default_log)),
    pairer(c.capture.pair_window_ms),
    lock_policy(c.capture.settle_frames, c.capture.lock_exposure_after_settle),
    rgb_throttle(c.rgb.fps, c.capture.sensor_mode.fps)
  {
    sides[0].impl = this;
    sides[0].side = CameraSide::left;
    sides[0].sensor_id = c.capture.left_sensor_id;
    sides[1].impl = this;
    sides[1].side = CameraSide::right;
    sides[1].sensor_id = c.capture.right_sensor_id;
  }

  std::string camera_label(const Side & s) const
  {
    return std::string(side_name(s.side)) + " camera (sensor-id " + std::to_string(s.sensor_id) + ")";
  }

  // ---- source pad probe: remember the sensor timestamp of each buffer by PTS ----
  static GstPadProbeReturn on_source_buffer(GstPad *, GstPadProbeInfo * info, gpointer user)
  {
    auto * s = static_cast<Side *>(user);
    GstBuffer * buf = GST_PAD_PROBE_INFO_BUFFER(info);
    static const GQuark quark = g_quark_from_static_string("GstBufferMetaData");
    const auto * aux = static_cast<const ArgusAuxData *>(gst_mini_object_get_qdata(GST_MINI_OBJECT_CAST(buf), quark));
    s->source_frames.fetch_add(1);
    if (aux && aux->timestamp > 0) s->last_sensor_ns.store(aux->timestamp);
    if (aux && GST_BUFFER_PTS_IS_VALID(buf)) {
      std::lock_guard<std::mutex> lock(s->ts_mutex);
      s->stamps[s->stamp_next] = {GST_BUFFER_PTS(buf), aux->timestamp, aux->frame_num};
      s->stamp_next = (s->stamp_next + 1) % kTimestampRing;
    }
    return GST_PAD_PROBE_OK;
  }

  static std::optional<Side::Stamp> lookup_stamp(Side & s, uint64_t pts)
  {
    std::lock_guard<std::mutex> lock(s.ts_mutex);
    for (const auto & st : s.stamps) {
      if (st.pts == pts && pts != GST_CLOCK_TIME_NONE) return st;
    }
    return std::nullopt;
  }

  // Capture time of a buffer that left the camera pipeline (its PTS survives nvvidconv, tee and nvjpegenc).
  std::optional<std::pair<int64_t, int64_t>> stamp_of(Side & s, GstBuffer * buf, uint64_t fallback_seq)
  {
    const uint64_t pts = GST_BUFFER_PTS(buf);
    const auto st = lookup_stamp(s, pts);
    std::optional<int64_t> sensor_ns;
    if (st) sensor_ns = st->sensor_ns;
    const auto mono = capture_monotonic_ns(sensor_ns, pts, gst_element_get_base_time(s.pipeline));
    if (!mono) return std::nullopt;
    const int64_t seq = st && st->frame_num >= 0 ? st->frame_num : static_cast<int64_t>(fallback_seq);
    return std::make_pair(clock.to_realtime(*mono), seq);
  }

  // ---- GRAY8 frames ----
  static GstFlowReturn on_gray_sample(GstAppSink * sink, gpointer user)
  {
    auto * s = static_cast<Side *>(user);
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;
    s->impl->handle_frame(*s, sample);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  // NVMM -> CUDA through EGL: the dmabuf of the NvBuffer becomes an EGLImage registered with CUDA; its pitch-linear
  // plane is wrapped in a GpuMat and copied device to device into the pooled frame. No CPU copy.
  bool map_nvmm(
    GstBuffer * buf, int rows, int cols, cv::cuda::GpuMat & dst, cv::cuda::Stream & stream, std::string & err)
  {
    if (!ensure_cuda_context()) {
      err = "no CUDA context";
      return false;
    }
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
      err = "gst_buffer_map failed";
      return false;
    }
    bool ok = false;
    int fd = -1;
    if (ExtractFdFromNvBuffer(map.data, &fd) != 0 || fd < 0) {
      err = "ExtractFdFromNvBuffer failed";
    } else {
      EGLImageKHR image = NvEGLImageFromFd(egl_display, fd);
      if (image == EGL_NO_IMAGE_KHR) {
        err = "NvEGLImageFromFd failed";
      } else {
        CUgraphicsResource resource = nullptr;
        CUresult r = cuGraphicsEGLRegisterImage(&resource, image, CU_GRAPHICS_MAP_RESOURCE_FLAGS_NONE);
        if (r != CUDA_SUCCESS) {
          err = "cuGraphicsEGLRegisterImage: " + cu_error(r);
        } else {
          CUeglFrame frame;
          r = cuGraphicsResourceGetMappedEglFrame(&frame, resource, 0, 0);
          if (r != CUDA_SUCCESS) {
            err = "cuGraphicsResourceGetMappedEglFrame: " + cu_error(r);
          } else if (frame.frameType != CU_EGL_FRAME_TYPE_PITCH) {
            err = "NVMM buffer is not pitch-linear";
          } else if (static_cast<int>(frame.width) < cols || static_cast<int>(frame.height) < rows) {
            err = "NVMM buffer smaller than the processing size";
          } else {
            const cv::cuda::GpuMat plane(rows, cols, CV_8UC1, frame.frame.pPitch[0], frame.pitch);
            plane.copyTo(dst, stream);
            stream.waitForCompletion();
            ok = true;
          }
          cuGraphicsUnregisterResource(resource);
        }
        NvDestroyEGLImage(egl_display, image);
      }
    }
    gst_buffer_unmap(buf, &map);
    return ok;
  }

  // System memory -> GPU upload (zero_copy false).
  static bool upload(
    GstSample * sample, GstBuffer * buf, int rows, int cols, cv::cuda::GpuMat & dst, cv::cuda::Stream & stream,
    std::string & err)
  {
    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample))) {
      err = "unreadable caps";
      return false;
    }
    if (GST_VIDEO_INFO_WIDTH(&info) != cols || GST_VIDEO_INFO_HEIGHT(&info) != rows) {
      err = "frame size differs from the processing size";
      return false;
    }
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
      err = "gst_buffer_map failed";
      return false;
    }
    const cv::Mat host(rows, cols, CV_8UC1, map.data, GST_VIDEO_INFO_PLANE_STRIDE(&info, 0));
    dst.upload(host, stream);
    stream.waitForCompletion();
    gst_buffer_unmap(buf, &map);
    return true;
  }

  void handle_frame(Side & s, GstSample * sample)
  {
    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf || !running.load()) return;
    ++s.frames;
    const auto stamp = stamp_of(s, buf, s.frames);
    if (!stamp) return;  // no usable time: the frame cannot be paired
    const int64_t now_mono = clock_ns(CLOCK_MONOTONIC);
    s.last_frame_mono_ns.store(now_mono);

    const int rows = config.processing.height;
    const int cols = config.processing.width;
    Frame frame;
    frame.capture_realtime_ns = stamp->first;
    frame.sequence = static_cast<uint64_t>(stamp->second);
    std::string err;
    bool converted = false;
    if (!s.pool.acquire(rows, cols, frame.image)) {
      err = "all " + std::to_string(kMaxPoolFrames) + " GPU frame buffers are in use";
    } else if (config.capture.zero_copy) {
      converted = map_nvmm(buf, rows, cols, frame.image, s.stream, err);
    } else {
      converted = upload(sample, buf, rows, cols, frame.image, s.stream, err);
    }
    if (!converted) {
      if (!s.convert_error_logged) log(camera_label(s) + ": frame dropped: " + err);
      s.convert_error_logged = true;
      return;  // never a placeholder frame
    }

    bool lock_now = false;
    bool read_back = false;
    bool in_error = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!s.error.empty() && !s.error_needs_restart) {
        log(camera_label(s) + ": frames resumed");
        s.error.clear();
      }
      lock_now = !aligning && lock_policy.on_frame(s.side);
      if (lock_now) frames_since_lock = 0;
      if (s.side == CameraSide::left && frames_since_lock >= 0 && ++frames_since_lock > kLockReadbackFrames) {
        frames_since_lock = -1;
        read_back = true;
      }
      in_error = !sides[0].error.empty() || !sides[1].error.empty();
    }
    if (lock_now) {
      for (auto & side : sides) g_object_set(side.src, "aelock", TRUE, "awblock", TRUE, nullptr);
      log(
        "AE/AWB settled after " + std::to_string(config.capture.settle_frames) +
        " frames per camera: aelock and awblock set on both cameras");
    }
    if (read_back) log_locked_exposure();

    auto pair = s.side == CameraSide::left ? pairer.push_left(std::move(frame)) : pairer.push_right(std::move(frame));
    if (!pair || in_error) return;  // no pair while a camera is in error
    {
      std::lock_guard<std::mutex> lock(mutex);
      any_pair = true;
    }
    if (pair_cb) pair_cb(*pair);
  }

  void log_locked_exposure()
  {
    std::ostringstream line;
    line << "exposure locked:";
    for (const auto & s : sides) {
      std::string err;
      const auto e = read_sensor_exposure(s.sensor_id, &err);
      line << " " << side_name(s.side) << " (sensor-id " << s.sensor_id << ")";
      if (e) {
        line << " exposure " << e->exposure_us << " us gain " << std::fixed << std::setprecision(2) << e->gain()
             << "x (raw " << e->gain_raw << ");";
      } else {
        line << " readback failed: " << err << ";";
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      exposure_locked = true;
    }
    log(line.str());
  }

  // ---- RGB JPEG branch ----
  // Keeps one buffer per 1/rgb.fps (half a sensor frame of tolerance) and drops the rest before nvvidconv and nvjpegenc
  // ever see them.
  static GstPadProbeReturn on_rgb_queue_buffer(GstPad *, GstPadProbeInfo * info, gpointer user)
  {
    auto * impl = static_cast<Impl *>(user);
    GstBuffer * buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!GST_BUFFER_PTS_IS_VALID(buf)) return GST_PAD_PROBE_DROP;
    return impl->rgb_throttle.admit(GST_BUFFER_PTS(buf)) ? GST_PAD_PROBE_OK : GST_PAD_PROBE_DROP;
  }

  struct MappedSample
  {
    GstSample * sample = nullptr;
    GstBuffer * buffer = nullptr;
    GstMapInfo map{};
    ~MappedSample()
    {
      if (buffer) gst_buffer_unmap(buffer, &map);
      if (sample) gst_sample_unref(sample);
    }
  };

  static GstFlowReturn on_rgb_sample(GstAppSink * sink, gpointer user)
  {
    auto * impl = static_cast<Impl *>(user);
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;
    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf || !impl->running.load()) {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }
    {
      std::lock_guard<std::mutex> lock(impl->mutex);
      if (!impl->sides[0].error.empty()) {  // the left camera is failing: no RGB from it
        gst_sample_unref(sample);
        return GST_FLOW_OK;
      }
    }
    auto mapped = std::make_shared<MappedSample>();
    mapped->sample = sample;
    mapped->buffer = buf;
    if (!gst_buffer_map(buf, &mapped->map, GST_MAP_READ)) {
      mapped->buffer = nullptr;
      gst_sample_unref(sample);
      mapped->sample = nullptr;
      return GST_FLOW_OK;
    }
    const auto stamp = impl->stamp_of(impl->sides[0], buf, impl->rgb_sequence + 1);
    if (!stamp) return GST_FLOW_OK;
    RgbJpeg jpeg;
    jpeg.data = mapped->map.data;
    jpeg.size = mapped->map.size;
    jpeg.capture_realtime_ns = stamp->first;
    jpeg.sequence = ++impl->rgb_sequence;
    jpeg.hold = mapped;
    if (impl->rgb_cb) impl->rgb_cb(jpeg);
    return GST_FLOW_OK;
  }

  // ---- lifecycle ----
  bool build_side(Side & s, std::string & err)
  {
    const std::string desc = build_pipeline_description(config, s.side);
    GError * gerr = nullptr;
    s.pipeline = gst_parse_launch(desc.c_str(), &gerr);
    if (!s.pipeline || gerr) {
      err = camera_label(s) + ": pipeline does not parse: " + (gerr ? gerr->message : "unknown error");
      if (gerr) g_error_free(gerr);
      return false;
    }
    const std::string name = side_name(s.side);
    s.src = gst_bin_get_by_name(GST_BIN(s.pipeline), (name + "_src").c_str());
    s.sink = gst_bin_get_by_name(GST_BIN(s.pipeline), (name + "_sink").c_str());
    if (!s.src || !s.sink) {
      err = camera_label(s) + ": pipeline lacks its source or sink";
      return false;
    }
    s.bus = gst_element_get_bus(s.pipeline);
    GstPad * pad = gst_element_get_static_pad(s.src, "src");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, on_source_buffer, &s, nullptr);
    gst_object_unref(pad);
    GstAppSinkCallbacks cbs{};
    cbs.new_sample = on_gray_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(s.sink), &cbs, &s, nullptr);

    if (s.side == CameraSide::left && config.rgb.enabled) {
      rgb_sink = gst_bin_get_by_name(GST_BIN(s.pipeline), "rgb_sink");
      rgb_jpeg = gst_bin_get_by_name(GST_BIN(s.pipeline), "rgb_jpeg");
      rgb_queue = gst_bin_get_by_name(GST_BIN(s.pipeline), "rgb_queue");
      if (!rgb_sink || !rgb_jpeg || !rgb_queue) {
        err = "left pipeline lacks the RGB branch";
        return false;
      }
      GstPad * qpad = gst_element_get_static_pad(rgb_queue, "sink");
      gst_pad_add_probe(qpad, GST_PAD_PROBE_TYPE_BUFFER, on_rgb_queue_buffer, this, nullptr);
      gst_object_unref(qpad);
      GstAppSinkCallbacks rcbs{};
      rcbs.new_sample = on_rgb_sample;
      gst_app_sink_set_callbacks(GST_APP_SINK(rgb_sink), &rcbs, this, nullptr);
    }
    return true;
  }

  bool play(Side & s, std::string & err)
  {
    const int64_t now = clock_ns(CLOCK_MONOTONIC);
    s.started_mono_ns.store(now);
    s.last_frame_mono_ns.store(0);
    s.last_sensor_ns.store(0);
    s.source_frames.store(0);
    if (s.side == CameraSide::left) rgb_throttle.reset();  // the pipeline is stopped: no probe is running
    if (gst_element_set_state(s.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      err = camera_label(s) + ": pipeline failed to start";
      return false;
    }
    return true;
  }

  void release()
  {
    for (auto & s : sides) {
      if (s.pipeline) gst_element_set_state(s.pipeline, GST_STATE_NULL);
    }
    for (GstElement ** e : {&rgb_sink, &rgb_jpeg, &rgb_queue}) {
      if (*e) gst_object_unref(*e);
      *e = nullptr;
    }
    for (auto & s : sides) {
      for (GstElement ** e : {&s.src, &s.sink, &s.pipeline}) {
        if (*e) gst_object_unref(*e);
        *e = nullptr;
      }
      if (s.bus) gst_object_unref(s.bus);
      s.bus = nullptr;
    }
  }

  void set_error(Side & s, const std::string & reason, bool needs_restart)
  {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (s.error.empty() || (needs_restart && !s.error_needs_restart)) {
        s.error = reason;
        s.error_needs_restart = needs_restart;
        changed = true;
      }
    }
    if (changed) {
      log("camera_error: " + reason);
      pairer.reset();  // frames queued before the failure never pair with frames after it
    }
  }

  // Restart one failed pipeline: both cameras unlock AE/AWB and settle again, so they lock together.
  void restart(Side & s, int64_t now)
  {
    std::string reason;
    {
      std::lock_guard<std::mutex> lock(mutex);
      s.last_restart_mono_ns = now;
      reason = s.error;
      s.error = "restarting after: " + reason;
      s.error_needs_restart = false;
      lock_policy.reset();
      exposure_locked = false;
      frames_since_lock = -1;
      aligning = true;
      align_attempts = 0;
    }
    log(camera_label(s) + ": restarting pipeline");
    gst_element_set_state(s.pipeline, GST_STATE_NULL);
    for (auto & side : sides) g_object_set(side.src, "aelock", FALSE, "awblock", FALSE, nullptr);
    std::string err;
    if (!play(s, err)) set_error(s, err, true);
  }

  // The sensors are not synchronised, and each one's frame phase is fixed when its stream starts (both run off the
  // Tegra MCLK, so it does not drift). Pairs need the phase within the pair window, so until it is within half the
  // window the right camera is restarted, which draws a new phase; after kMaxAlignAttempts it pairs at whatever phase
  // it has (the drops are counted and the skew histogram shows it).
  void align_step()
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!aligning || !sides[0].error.empty() || !sides[1].error.empty()) return;
    }
    auto & left = sides[0];
    auto & right = sides[1];
    if (left.source_frames.load() < kAlignFrames || right.source_frames.load() < kAlignFrames) return;
    const int64_t lt = left.last_sensor_ns.load();
    const int64_t rt = right.last_sensor_ns.load();
    const int64_t period = static_cast<int64_t>(1e9 / config.capture.sensor_mode.fps);
    std::ostringstream msg;
    if (lt == 0 || rt == 0) {
      msg << "no sensor timestamps from nvarguscamerasrc: camera phase not aligned";
    } else {
      const int64_t phase = (((rt - lt) % period) + period) % period;
      const int64_t skew = std::min(phase, period - phase);
      msg << std::fixed << std::setprecision(2) << "camera phase skew " << skew / 1e6 << " ms after " << align_attempts
          << " right-camera restarts";
      if (2 * skew > static_cast<int64_t>(config.capture.pair_window_ms * 1e6) && align_attempts < kMaxAlignAttempts) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          ++align_attempts;
        }
        gst_element_set_state(right.pipeline, GST_STATE_NULL);
        std::this_thread::sleep_for(std::chrono::milliseconds((align_attempts * 37) % 100));
        std::string err;
        if (!play(right, err)) set_error(right, err, true);
        pairer.reset();
        return;
      }
      if (2 * skew > static_cast<int64_t>(config.capture.pair_window_ms * 1e6)) {
        msg << ": not within half of pair_window_ms, pairing anyway";
      } else {
        msg << ": aligned";
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      aligning = false;
    }
    log(msg.str());
  }

  void watch()
  {
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(kWatchdogTickNs));
      const int64_t now = clock_ns(CLOCK_MONOTONIC);
      for (auto & s : sides) {
        while (GstMessage * msg =
                 gst_bus_pop_filtered(s.bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS))) {
          std::string reason = camera_label(s) + ": ";
          if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError * gerr = nullptr;
            gst_message_parse_error(msg, &gerr, nullptr);
            reason += gerr ? gerr->message : "pipeline error";
            if (gerr) g_error_free(gerr);
          } else {
            reason += "stream ended";
          }
          gst_message_unref(msg);
          set_error(s, reason, true);
        }
        const int64_t last = s.last_frame_mono_ns.load();
        const int64_t since = last ? last : s.started_mono_ns.load();
        if (camera_timed_out(now, last, s.started_mono_ns.load())) {
          std::ostringstream reason;
          reason << camera_label(s) << " delivered no frame for " << std::fixed << std::setprecision(1)
                 << (now - since) / 1e9 << " s";
          set_error(s, reason.str(), true);
        }
        bool restart_due = false;
        {
          std::lock_guard<std::mutex> lock(mutex);
          restart_due = s.error_needs_restart && now - s.last_restart_mono_ns >= kRestartBackoffNs;
        }
        if (restart_due && running.load()) restart(s, now);
      }
      if (running.load()) align_step();
    }
  }
};

StereoCapture::StereoCapture(const Config & config, LogFn log) : impl_(std::make_unique<Impl>(config, std::move(log)))
{
}

StereoCapture::~StereoCapture() { stop(); }

void StereoCapture::set_pair_callback(PairCallback cb) { impl_->pair_cb = std::move(cb); }

void StereoCapture::set_rgb_callback(RgbCallback cb) { impl_->rgb_cb = std::move(cb); }

bool StereoCapture::start(std::string * error)
{
  auto & im = *impl_;
  if (im.running.load()) return true;
  auto fail = [&](const std::string & why) {
    {
      std::lock_guard<std::mutex> lock(im.mutex);
      im.start_error = why;
    }
    im.release();
    im.log("camera_error: " + why);
    if (error) *error = why;
    return false;
  };
  try {
    build_pipeline_description(im.config, CameraSide::left);
  } catch (const std::invalid_argument & e) {
    return fail(e.what());
  }
  gst_init(nullptr, nullptr);
  if (im.config.rgb.enabled) bind_nvjpeg_plugin(im.log);
  if (im.config.capture.zero_copy) im.egl_display = shared_egl_display();
  std::string err;
  for (auto & s : im.sides) {
    if (!im.build_side(s, err)) return fail(err);
  }
  {
    std::lock_guard<std::mutex> lock(im.mutex);
    im.start_error.clear();
  }
  im.running.store(true);
  for (auto & s : im.sides) {
    if (!im.play(s, err)) {
      im.running.store(false);
      return fail(err);
    }
  }
  im.watchdog = std::thread([&im] { im.watch(); });
  im.log(
    "started: left sensor-id " + std::to_string(im.config.capture.left_sensor_id) + ", right sensor-id " +
    std::to_string(im.config.capture.right_sensor_id) + (im.config.capture.zero_copy ? ", zero copy" : ", upload"));
  return true;
}

void StereoCapture::stop()
{
  auto & im = *impl_;
  const bool was_running = im.running.exchange(false);
  if (im.watchdog.joinable()) im.watchdog.join();
  if (was_running) im.release();
}

CaptureStatus StereoCapture::status() const
{
  const auto & im = *impl_;
  std::lock_guard<std::mutex> lock(im.mutex);
  CaptureStatus st;
  st.exposure_locked = im.exposure_locked;
  if (!im.start_error.empty()) {
    st.state = CaptureState::camera_error;
    st.reason = im.start_error;
    st.left_error = st.right_error = true;
    return st;
  }
  st.left_error = !im.sides[0].error.empty();
  st.right_error = !im.sides[1].error.empty();
  std::string reasons;
  for (const auto & s : im.sides) {
    if (s.error.empty()) continue;
    if (!reasons.empty()) reasons += "; ";
    reasons += s.error;
  }
  if (!reasons.empty()) {
    st.state = CaptureState::camera_error;
    st.reason = reasons;
  } else if (!im.running.load()) {
    st.state = CaptureState::starting;
    st.reason = "not started";
  } else if (im.aligning) {
    st.state = CaptureState::starting;
    st.reason = "aligning the camera phase (" + std::to_string(im.align_attempts) + " restarts so far)";
  } else if (!im.any_pair) {
    st.state = CaptureState::starting;
    st.reason = "waiting for the first stereo pair";
  } else {
    st.state = CaptureState::ok;
  }
  return st;
}

PairingStats StereoCapture::pairing_stats() const { return impl_->pairer.stats(); }

void StereoCapture::set_jpeg_quality(int quality)
{
  impl_->config.rgb.jpeg_quality = quality;
  if (impl_->rgb_jpeg) g_object_set(impl_->rgb_jpeg, "quality", static_cast<gint>(quality), nullptr);
}

}  // namespace depthd
