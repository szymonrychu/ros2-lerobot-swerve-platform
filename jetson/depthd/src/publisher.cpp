#include "depthd/publisher.hpp"

#include <zmq.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

namespace depthd
{
namespace wire = ros2_common_cpp::depth_wire;

namespace
{
void set_int(void * socket, int option, int value, const char * name)
{
  if (zmq_setsockopt(socket, option, &value, sizeof(value)) != 0) {
    throw std::runtime_error(std::string("zmq_setsockopt ") + name + ": " + zmq_strerror(zmq_errno()));
  }
}
}  // namespace

Publisher::Publisher(const Transport & transport) : zstd_level_(transport.zstd_level)
{
  ctx_ = zmq_ctx_new();
  if (ctx_ == nullptr) {
    throw std::runtime_error("zmq_ctx_new failed");
  }
  try {
    socket_ = zmq_socket(ctx_, ZMQ_PUB);
    if (socket_ == nullptr) {
      throw std::runtime_error(std::string("zmq_socket: ") + zmq_strerror(zmq_errno()));
    }
    set_int(socket_, ZMQ_SNDHWM, transport.send_hwm, "ZMQ_SNDHWM");
    set_int(socket_, ZMQ_CONFLATE, 0, "ZMQ_CONFLATE");
    set_int(socket_, ZMQ_XPUB_NODROP, 1, "ZMQ_XPUB_NODROP");
    set_int(socket_, ZMQ_LINGER, 0, "ZMQ_LINGER");
    const std::string endpoint = "tcp://" + transport.bind_address + ":" + std::to_string(transport.pub_port);
    if (zmq_bind(socket_, endpoint.c_str()) != 0) {
      throw std::runtime_error("zmq_bind " + endpoint + ": " + zmq_strerror(zmq_errno()));
    }
    char last[256] = {0};
    size_t size = sizeof(last);
    if (zmq_getsockopt(socket_, ZMQ_LAST_ENDPOINT, last, &size) != 0) {
      throw std::runtime_error(std::string("zmq_getsockopt ZMQ_LAST_ENDPOINT: ") + zmq_strerror(zmq_errno()));
    }
    const char * colon = std::strrchr(last, ':');
    port_ = colon != nullptr ? std::atoi(colon + 1) : transport.pub_port;
  } catch (...) {
    if (socket_ != nullptr) {
      zmq_close(socket_);
    }
    zmq_ctx_term(ctx_);
    throw;
  }
}

Publisher::~Publisher()
{
  zmq_close(socket_);
  zmq_ctx_term(ctx_);
}

bool Publisher::send_frames(const wire::Frames & frames)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  const auto send_one = [this](const void * data, size_t size, int flags) {
    return zmq_send(socket_, data, size, flags | ZMQ_DONTWAIT) >= 0;
  };
  // A multipart message is queued atomically: when the first frame is accepted the rest follow, and when the high
  // water mark is full the first frame is refused (EAGAIN) and nothing of the message is queued.
  if (!send_one(frames.topic.data(), frames.topic.size(), ZMQ_SNDMORE)) {
    ++drops_;
    return false;
  }
  if (!send_one(frames.header.data(), frames.header.size(), ZMQ_SNDMORE) ||
      !send_one(frames.payload.data(), frames.payload.size(), 0)) {
    ++drops_;
    return false;
  }
  ++sent_;
  return true;
}

bool Publisher::send_depth(wire::Header header, const std::vector<uint16_t> & depth_mm)
{
  header.kind = wire::Kind::kDepth;
  try {
    return send_frames(wire::encode_depth(header, depth_mm, zstd_level_));
  } catch (const std::exception &) {
    return false;
  }
}

bool Publisher::send_fused(wire::Header header, const std::vector<uint16_t> & depth_mm)
{
  header.kind = wire::Kind::kFused;
  try {
    return send_frames(wire::encode_depth(header, depth_mm, zstd_level_));
  } catch (const std::exception &) {
    return false;
  }
}

bool Publisher::send_rgb(wire::Header header, const std::vector<uint8_t> & jpeg)
{
  header.kind = wire::Kind::kRgb;
  try {
    return send_frames(wire::encode_rgb(header, jpeg));
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace depthd
