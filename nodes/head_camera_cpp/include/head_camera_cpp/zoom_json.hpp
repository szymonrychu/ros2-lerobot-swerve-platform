#pragma once

// The zoom interface contract (design D4), defined once here; the node README mirrors it.
//
// Three topics, all under the node's topic namespace (default /head_camera):
//
//   /head_camera/zoom/command  std_msgs/String, JSON request. QoS reliable, keep_last 10.
//     {"request_id": "<str>", "u": 0.62, "v": 0.41, "zoom": 4, "max_px": 1024, "af": true}
//       request_id  string, required, non-empty. Echoed in the result. A command without a usable request_id is
//                   logged and dropped (no result can be addressed to it).
//       u, v        numbers in [0, 1], required. Normalised point in the default view, top-left origin.
//       zoom        integer, required, one of ZoomConfig::factors (2, 4 or 8).
//       max_px      integer in [ZoomConfig::min_max_px, ZoomConfig::max_max_px] (256..1024), default
//                   ZoomConfig::default_max_px (1024). Longest side of the output image.
//       af          bool, default true. Run windowed autofocus before the still.
//     Unknown keys are rejected.
//
//   /head_camera/zoom/image    sensor_msgs/CompressedImage, JPEG, longest side <= max_px, header.stamp = the source
//     frame's capture stamp, frame_id head_camera_optical_frame. QoS reliable, keep_last 2.
//     Published BEFORE the result. Only statuses for which carries_image() is true have one.
//
//   /head_camera/zoom/result   std_msgs/String, JSON result. QoS reliable, keep_last 10. Exactly these keys:
//     {"request_id": str, "status": str, "message": str, "af_state": str, "lens_position": number|null,
//      "crop": [x, y, w, h]|null, "full": [w, h], "out": [w, h]|null, "stamp_ns": int, "stamp_s": number,
//      "phase_ms": {"af": number, "switch": number, "capture": number, "restore": number}}
//       crop         the region taken from the full-resolution frame; full: that frame's size (4608x2592).
//       out          the JPEG's size. null where there is no image.
//       lens_position the lens position (dioptres) after the capture, null when unknown.
//       stamp_ns     equals the image header stamp exactly (consumers pair image and result by it, since a
//                    CompressedImage carries no request id); 0 where there is no image.
//       stamp_s      stamp_ns / 1e9.
//   status:
//     ok            image taken.                                   carries an image
//     af_failed     focus did not converge; the picture is taken.  carries an image
//     busy          another zoom is running.                       no image
//     timeout       a phase timed out.                             no image
//     camera_error  the camera failed.                             no image
//     invalid       malformed fields, request_id present.          no image
//
// Serves G2, U3.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "head_camera_cpp/config.hpp"

namespace head_camera_cpp
{

// A validated zoom request.
struct ZoomRequest
{
  std::string request_id;
  double u = 0.0;
  double v = 0.0;
  int zoom = 0;
  int max_px = 0;
  bool af = true;
};

// A rejected command. request_id is set when the command named a usable one (answer with invalid_result), unset
// when it did not (log and drop).
struct ParseError
{
  std::optional<std::string> request_id;
  std::string message;
};

enum class ZoomStatus
{
  Ok,
  AfFailed,
  Busy,
  Timeout,
  CameraError,
  Invalid
};

// Duration of each zoom phase in milliseconds; 0 for a phase that did not run.
struct PhaseMs
{
  double af = 0.0;
  double switch_mode = 0.0;  // JSON key "switch"
  double capture = 0.0;
  double restore = 0.0;
};

struct ZoomResult
{
  std::string request_id;
  ZoomStatus status = ZoomStatus::Invalid;
  std::string message;
  std::string af_state;
  std::optional<double> lens_position;
  std::optional<std::array<int, 4>> crop;  // x, y, w, h in the full frame
  std::array<int, 2> full{4608, 2592};
  std::optional<std::array<int, 2>> out;  // w, h of the JPEG
  int64_t stamp_ns = 0;
  PhaseMs phase_ms;
};

// Parse and validate a command.
// @param json the std_msgs/String payload.
// @param limits allowed zoom factors and max_px range and default.
// @return the request, or a ParseError.
std::variant<ZoomRequest, ParseError> parse_request(const std::string & json, const ZoomConfig & limits);

// @param status a status.
// @return its wire name (ok, af_failed, busy, timeout, camera_error, invalid).
std::string to_string(ZoomStatus status);

// @param status a status.
// @return true for ok and af_failed: the statuses that publish an image.
bool carries_image(ZoomStatus status);

// @param result a result.
// @return the JSON text with exactly the contract keys; absent optionals are null.
std::string to_json(const ZoomResult & result);

// @param error a ParseError that carries a request_id.
// @return the invalid result answering it.
ZoomResult invalid_result(const ParseError & error);

}  // namespace head_camera_cpp
