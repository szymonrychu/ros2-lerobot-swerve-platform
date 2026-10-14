// Depth wire format: the one definition shared by jetson/depthd (publisher) and nodes/jetson_bridge_cpp (subscriber).
// Depends only on the C++17 standard library and libzstd (no rclcpp, OpenCV or ZeroMQ); builds with gcc 8.4 and Jazzy.
//
// A message is ZeroMQ multipart: [topic frame, header frame, payload frame].
//   topic    "depth" | "fused": payload is a zstd frame of width*height little-endian uint16 millimetres (0 = invalid)
//            "rgb": payload is one JPEG, carried unchanged
//   header   fixed-size little-endian layout, see kHeaderSize and README.md; written field by field
#ifndef ROS2_COMMON_CPP__DEPTH_WIRE_HPP_
#define ROS2_COMMON_CPP__DEPTH_WIRE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ros2_common_cpp
{
namespace depth_wire
{

constexpr char kMagic[4] = {'D', 'P', 'W', '1'};
constexpr uint16_t kVersion = 1;
constexpr size_t kMaxFrameIdBytes = 63;
constexpr uint32_t kMaxDimension = 16384;
// 4 magic + 2 version + 1 kind + 1 reserved + 8 sequence + 8 capture + 8 lr + 4 w + 4 h + 1 + 63 frame_id
// + 9*8 K + 5*8 D + 12*8 P + 8 baseline + 1 calibrated + 7 pad + 7*4 floats + 3*8 counters
constexpr size_t kHeaderSize = 4 + 2 + 1 + 1 + 8 + 8 + 8 + 4 + 4 + 1 + 63 + 72 + 40 + 96 + 8 + 1 + 7 + 28 + 24;

constexpr const char * kTopicDepth = "depth";
constexpr const char * kTopicFused = "fused";
constexpr const char * kTopicRgb = "rgb";

enum class Kind : uint8_t { kDepth = 0, kFused = 1, kRgb = 2 };

// Topic frame of a kind ("depth", "fused", "rgb").
const char * topic_of(Kind kind);

struct Header
{
  Kind kind = Kind::kDepth;
  uint64_t sequence = 0;
  int64_t capture_realtime_ns = 0;   // left sensor capture, CLOCK_REALTIME
  int64_t right_minus_left_ns = 0;   // right capture time minus left capture time
  uint32_t width = 0;
  uint32_t height = 0;
  std::string frame_id;              // at most 63 bytes
  std::array<double, 9> k{};         // camera matrix
  std::array<double, 5> d{};         // distortion
  std::array<double, 12> p{};        // rectified projection
  double baseline_m = 0.0;
  bool calibrated = false;
  // Jetson health
  float pipeline_fps = 0.0F;
  float stage_capture_ms = 0.0F;
  float stage_rectify_ms = 0.0F;
  float stage_match_ms = 0.0F;
  float stage_post_ms = 0.0F;
  float stage_mono_ms = 0.0F;
  float stage_encode_ms = 0.0F;
  uint64_t pairs_dropped_total = 0;
  float gpu_temp_c = 0.0F;
  uint64_t nvgpu_errors_total = 0;
};

// The three frames of one message.
struct Frames
{
  std::string topic;
  std::vector<uint8_t> header;
  std::vector<uint8_t> payload;
};

// A decoded message: depth_mm for depth/fused, jpeg for rgb. error is empty on success; on failure header is
// default-constructed and the payload vectors are empty (never a partial result).
struct Decoded
{
  std::string error;
  Header header;
  std::vector<uint16_t> depth_mm;
  std::vector<uint8_t> jpeg;
  bool ok() const {return error.empty();}
};

// Header codec. encode throws std::invalid_argument when frame_id exceeds 63 bytes. decode returns an empty string
// on success, else the reason, and leaves out untouched on failure.
std::vector<uint8_t> encode_header(const Header & header);
std::string decode_header(const uint8_t * data, size_t size, Header & out);

// depth/fused message: header.kind must be kDepth or kFused (else std::invalid_argument); width*height must equal
// depth_mm.size(). The payload is zstd at the given level.
Frames encode_depth(const Header & header, const std::vector<uint16_t> & depth_mm, int zstd_level = 1);
// rgb message: header.kind must be kRgb; the JPEG bytes are copied unchanged.
Frames encode_rgb(const Header & header, const std::vector<uint8_t> & jpeg);

// Decode three received frames. Rejects an unknown topic, wrong magic, unknown version, truncated or oversized
// header, a header kind that does not match the topic, and a depth payload whose decompressed size is not
// width*height*2.
Decoded decode(const std::string & topic, const uint8_t * header, size_t header_size, const uint8_t * payload,
  size_t payload_size);
Decoded decode(const Frames & frames);

}  // namespace depth_wire
}  // namespace ros2_common_cpp

#endif  // ROS2_COMMON_CPP__DEPTH_WIRE_HPP_
