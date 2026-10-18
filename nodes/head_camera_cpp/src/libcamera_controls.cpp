// Pure libcamera helpers of the adapter: the control list of each phase and the type conversions. No camera.

#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>

#include <algorithm>
#include <cstdint>

#include "head_camera_cpp/libcamera_backend.hpp"

namespace head_camera_cpp
{

namespace
{

/// @return The libcamera AfMode of a validated config af_mode (continuous, auto or manual).
int32_t af_mode_value(const std::string & af_mode)
{
  if (af_mode == "auto") {
    return libcamera::controls::AfModeAuto;
  }
  if (af_mode == "manual") {
    return libcamera::controls::AfModeManual;
  }
  return libcamera::controls::AfModeContinuous;
}

}  // namespace

libcamera::ControlList focus_controls(const Config & config, bool locked)
{
  libcamera::ControlList list(libcamera::controls::controls);
  const int32_t mode = locked ? libcamera::controls::AfModeManual : af_mode_value(config.af_mode);
  list.set(libcamera::controls::AfMode, mode);
  if (mode == libcamera::controls::AfModeManual) {
    list.set(libcamera::controls::LensPosition, static_cast<float>(config.lens_position));
  }
  return list;
}

libcamera::ControlList video_controls(const Config & config, bool focus_locked)
{
  libcamera::ControlList list = focus_controls(config, focus_locked);
  const auto frame_duration = static_cast<int64_t>(config.frame_duration_us);
  list.set(libcamera::controls::FrameDurationLimits, {frame_duration, frame_duration});
  return list;
}

libcamera::ControlList af_controls(const AfRequest & request)
{
  libcamera::ControlList list(libcamera::controls::controls);
  list.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
  if (request.windowed) {
    const libcamera::Rectangle window = to_rectangle(request.window);
    list.set(libcamera::controls::AfMetering, libcamera::controls::AfMeteringWindows);
    list.set(libcamera::controls::AfWindows, libcamera::Span<const libcamera::Rectangle>(&window, 1));
  } else {
    list.set(libcamera::controls::AfMetering, libcamera::controls::AfMeteringAuto);
  }
  if (request.trigger) {
    list.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerStart);
  }
  return list;
}

libcamera::ControlList still_controls()
{
  libcamera::ControlList list(libcamera::controls::controls);
  list.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
  return list;
}

AfState to_af_state(int32_t libcamera_af_state)
{
  switch (libcamera_af_state) {
    case libcamera::controls::AfStateScanning:
      return AfState::Scanning;
    case libcamera::controls::AfStateFocused:
      return AfState::Focused;
    case libcamera::controls::AfStateFailed:
      return AfState::Failed;
    case libcamera::controls::AfStateIdle:
    default:
      return AfState::Idle;
  }
}

libcamera::Rectangle to_rectangle(Rect rect)
{
  return libcamera::Rectangle(
    rect.x, rect.y, static_cast<unsigned int>(std::max(rect.width, 0)),
    static_cast<unsigned int>(std::max(rect.height, 0)));
}

Rect from_rectangle(const libcamera::Rectangle & rectangle)
{
  return Rect{rectangle.x, rectangle.y, static_cast<int>(rectangle.width), static_cast<int>(rectangle.height)};
}

}  // namespace head_camera_cpp
