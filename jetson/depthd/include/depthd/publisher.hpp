// ZeroMQ PUB side of the depth link (libzmq C API; focal ships no cppzmq).
//
// One PUB socket on tcp://<transport.bind_address>:<transport.pub_port>. Each message is the three frames that
// ros2_common_cpp::depth_wire encodes (topic, header, payload); nothing else is written to the wire. The pipeline
// never waits on the link: every send is ZMQ_DONTWAIT and ZMQ_XPUB_NODROP turns a full send high-water mark
// (transport.send_hwm, ZMQ_CONFLATE off so frames queue in order) into EAGAIN, which is dropped and counted in
// send_drops() (exported as depthd_zmq_send_drops_total).
#ifndef DEPTHD_PUBLISHER_HPP_
#define DEPTHD_PUBLISHER_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include "depthd/config.hpp"
#include "ros2_common_cpp/depth_wire.hpp"

namespace depthd
{

class Publisher
{
public:
  // Binds the socket; throws std::runtime_error naming the endpoint on failure. pub_port 0 binds any free port.
  explicit Publisher(const Transport & transport);
  ~Publisher();
  Publisher(const Publisher &) = delete;
  Publisher & operator=(const Publisher &) = delete;

  // Each returns true when the message was queued. False when it was dropped because the send high-water mark is
  // full (counted in send_drops()) or when it could not be encoded (width*height mismatch, frame_id too long: not
  // counted). The header's kind is set by the method. Safe to call from several threads.
  bool send_depth(ros2_common_cpp::depth_wire::Header header, const std::vector<uint16_t> & depth_mm);
  bool send_fused(ros2_common_cpp::depth_wire::Header header, const std::vector<uint16_t> & depth_mm);
  bool send_rgb(ros2_common_cpp::depth_wire::Header header, const std::vector<uint8_t> & jpeg);

  int bound_port() const { return port_; }
  uint64_t send_drops() const { return drops_.load(); }
  uint64_t sent() const { return sent_.load(); }

private:
  bool send_frames(const ros2_common_cpp::depth_wire::Frames & frames);

  void * ctx_ = nullptr;
  void * socket_ = nullptr;
  int port_ = 0;
  int zstd_level_ = 1;
  std::mutex mutex_;  // zmq sockets are not thread safe; a DONTWAIT send holds it only briefly
  std::atomic<uint64_t> drops_{0};
  std::atomic<uint64_t> sent_{0};
};

}  // namespace depthd

#endif  // DEPTHD_PUBLISHER_HPP_
