#include "ros2_common_cpp/depth_wire.hpp"

#include <zstd.h>

#include <cstring>
#include <stdexcept>

namespace ros2_common_cpp
{
namespace depth_wire
{
namespace
{

class Writer
{
public:
  explicit Writer(std::vector<uint8_t> & out)
  : out_(out) {}
  void u8(uint8_t v) {out_.push_back(v);}
  void le(uint64_t v, int bytes)
  {
    for (int i = 0; i < bytes; ++i) {
      out_.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFFU));
    }
  }
  void u16(uint16_t v) {le(v, 2);}
  void u32(uint32_t v) {le(v, 4);}
  void u64(uint64_t v) {le(v, 8);}
  void i64(int64_t v) {u64(static_cast<uint64_t>(v));}
  void f32(float v)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof bits);
    u32(bits);
  }
  void f64(double v)
  {
    uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof bits);
    u64(bits);
  }
  void zeros(size_t n) {out_.insert(out_.end(), n, 0);}

private:
  std::vector<uint8_t> & out_;
};

class Reader
{
public:
  Reader(const uint8_t * data, size_t size)
  : data_(data), size_(size) {}
  uint64_t le(int bytes)
  {
    uint64_t v = 0;
    for (int i = 0; i < bytes; ++i) {
      v |= static_cast<uint64_t>(data_[pos_ + i]) << (8 * i);
    }
    pos_ += bytes;
    return v;
  }
  uint8_t u8() {return static_cast<uint8_t>(le(1));}
  uint16_t u16() {return static_cast<uint16_t>(le(2));}
  uint32_t u32() {return static_cast<uint32_t>(le(4));}
  uint64_t u64() {return le(8);}
  int64_t i64() {return static_cast<int64_t>(u64());}
  float f32()
  {
    const uint32_t bits = u32();
    float v = 0;
    std::memcpy(&v, &bits, sizeof v);
    return v;
  }
  double f64()
  {
    const uint64_t bits = u64();
    double v = 0;
    std::memcpy(&v, &bits, sizeof v);
    return v;
  }
  const uint8_t * here() const {return data_ + pos_;}
  void skip(size_t n) {pos_ += n;}
  size_t size() const {return size_;}

private:
  const uint8_t * data_;
  size_t size_;
  size_t pos_ = 0;
};

bool kind_from_topic(const std::string & topic, Kind & kind)
{
  if (topic == kTopicDepth) {
    kind = Kind::kDepth;
  } else if (topic == kTopicFused) {
    kind = Kind::kFused;
  } else if (topic == kTopicRgb) {
    kind = Kind::kRgb;
  } else {
    return false;
  }
  return true;
}

Decoded failure(const std::string & why)
{
  Decoded d;
  d.error = why;
  return d;
}

}  // namespace

const char * topic_of(Kind kind)
{
  switch (kind) {
    case Kind::kDepth: return kTopicDepth;
    case Kind::kFused: return kTopicFused;
    case Kind::kRgb: return kTopicRgb;
  }
  return "";
}

std::vector<uint8_t> encode_header(const Header & h)
{
  if (h.frame_id.size() > kMaxFrameIdBytes) {
    throw std::invalid_argument("frame_id longer than 63 bytes");
  }
  std::vector<uint8_t> out;
  out.reserve(kHeaderSize);
  Writer w(out);
  for (char c : kMagic) {
    w.u8(static_cast<uint8_t>(c));
  }
  w.u16(kVersion);
  w.u8(static_cast<uint8_t>(h.kind));
  w.u8(0);
  w.u64(h.sequence);
  w.i64(h.capture_realtime_ns);
  w.i64(h.right_minus_left_ns);
  w.u32(h.width);
  w.u32(h.height);
  w.u8(static_cast<uint8_t>(h.frame_id.size()));
  for (char c : h.frame_id) {
    w.u8(static_cast<uint8_t>(c));
  }
  w.zeros(kMaxFrameIdBytes - h.frame_id.size());
  for (double v : h.k) {w.f64(v);}
  for (double v : h.d) {w.f64(v);}
  for (double v : h.p) {w.f64(v);}
  w.f64(h.baseline_m);
  w.u8(h.calibrated ? 1 : 0);
  w.zeros(7);
  w.f32(h.pipeline_fps);
  w.f32(h.stage_capture_ms);
  w.f32(h.stage_rectify_ms);
  w.f32(h.stage_match_ms);
  w.f32(h.stage_post_ms);
  w.f32(h.stage_mono_ms);
  w.f32(h.stage_encode_ms);
  w.u64(h.pairs_dropped_total);
  w.f32(h.gpu_temp_c);
  w.zeros(4);
  w.u64(h.nvgpu_errors_total);
  return out;
}

std::string decode_header(const uint8_t * data, size_t size, Header & out)
{
  if (data == nullptr || size < kHeaderSize) {
    return "truncated header";
  }
  if (size != kHeaderSize) {
    return "header size mismatch";
  }
  Reader r(data, size);
  if (std::memcmp(r.here(), kMagic, sizeof kMagic) != 0) {
    return "wrong magic";
  }
  r.skip(sizeof kMagic);
  if (r.u16() != kVersion) {
    return "unknown version";
  }
  Header h;
  const uint8_t kind = r.u8();
  if (kind > static_cast<uint8_t>(Kind::kRgb)) {
    return "unknown kind";
  }
  h.kind = static_cast<Kind>(kind);
  r.skip(1);
  h.sequence = r.u64();
  h.capture_realtime_ns = r.i64();
  h.right_minus_left_ns = r.i64();
  h.width = r.u32();
  h.height = r.u32();
  const size_t id_len = r.u8();
  if (id_len > kMaxFrameIdBytes) {
    return "frame_id length out of range";
  }
  h.frame_id.assign(reinterpret_cast<const char *>(r.here()), id_len);
  r.skip(kMaxFrameIdBytes);
  for (double & v : h.k) {v = r.f64();}
  for (double & v : h.d) {v = r.f64();}
  for (double & v : h.p) {v = r.f64();}
  h.baseline_m = r.f64();
  h.calibrated = r.u8() != 0;
  r.skip(7);
  h.pipeline_fps = r.f32();
  h.stage_capture_ms = r.f32();
  h.stage_rectify_ms = r.f32();
  h.stage_match_ms = r.f32();
  h.stage_post_ms = r.f32();
  h.stage_mono_ms = r.f32();
  h.stage_encode_ms = r.f32();
  h.pairs_dropped_total = r.u64();
  h.gpu_temp_c = r.f32();
  r.skip(4);
  h.nvgpu_errors_total = r.u64();
  out = h;
  return "";
}

Frames encode_depth(const Header & header, const std::vector<uint16_t> & depth_mm, int zstd_level)
{
  if (header.kind != Kind::kDepth && header.kind != Kind::kFused) {
    throw std::invalid_argument("encode_depth needs kind depth or fused");
  }
  if (static_cast<uint64_t>(header.width) * header.height != depth_mm.size()) {
    throw std::invalid_argument("width*height does not match the depth image size");
  }
  std::vector<uint8_t> raw(depth_mm.size() * 2);
  for (size_t i = 0; i < depth_mm.size(); ++i) {
    raw[2 * i] = static_cast<uint8_t>(depth_mm[i] & 0xFFU);
    raw[2 * i + 1] = static_cast<uint8_t>(depth_mm[i] >> 8);
  }
  Frames f;
  f.topic = topic_of(header.kind);
  f.header = encode_header(header);
  f.payload.resize(ZSTD_compressBound(raw.size()));
  const size_t n = ZSTD_compress(f.payload.data(), f.payload.size(), raw.data(), raw.size(), zstd_level);
  if (ZSTD_isError(n)) {
    throw std::runtime_error(std::string("zstd compress failed: ") + ZSTD_getErrorName(n));
  }
  f.payload.resize(n);
  return f;
}

Frames encode_rgb(const Header & header, const std::vector<uint8_t> & jpeg)
{
  if (header.kind != Kind::kRgb) {
    throw std::invalid_argument("encode_rgb needs kind rgb");
  }
  Frames f;
  f.topic = kTopicRgb;
  f.header = encode_header(header);
  f.payload = jpeg;
  return f;
}

Decoded decode(const std::string & topic, const uint8_t * header, size_t header_size, const uint8_t * payload,
  size_t payload_size)
{
  Kind topic_kind;
  if (!kind_from_topic(topic, topic_kind)) {
    return failure("unknown topic");
  }
  Decoded d;
  d.error = decode_header(header, header_size, d.header);
  if (!d.error.empty()) {
    return failure(d.error);
  }
  if (d.header.kind != topic_kind) {
    return failure("header kind does not match topic");
  }
  if (topic_kind == Kind::kRgb) {
    d.jpeg.assign(payload, payload + payload_size);
    return d;
  }
  if (d.header.width == 0 || d.header.height == 0 || d.header.width > kMaxDimension ||
    d.header.height > kMaxDimension)
  {
    return failure("width or height out of range");
  }
  const uint64_t expected = static_cast<uint64_t>(d.header.width) * d.header.height * 2;
  const unsigned long long content = ZSTD_getFrameContentSize(payload, payload_size);
  if (content == ZSTD_CONTENTSIZE_ERROR || content == ZSTD_CONTENTSIZE_UNKNOWN || content != expected) {
    return failure("width*height does not match the decompressed payload");
  }
  std::vector<uint8_t> raw(expected);
  const size_t n = ZSTD_decompress(raw.data(), raw.size(), payload, payload_size);
  if (ZSTD_isError(n) || n != expected) {
    return failure("width*height does not match the decompressed payload");
  }
  d.depth_mm.resize(expected / 2);
  for (size_t i = 0; i < d.depth_mm.size(); ++i) {
    d.depth_mm[i] = static_cast<uint16_t>(raw[2 * i] | (raw[2 * i + 1] << 8));
  }
  return d;
}

Decoded decode(const Frames & f)
{
  return decode(f.topic, f.header.data(), f.header.size(), f.payload.data(), f.payload.size());
}

}  // namespace depth_wire
}  // namespace ros2_common_cpp
