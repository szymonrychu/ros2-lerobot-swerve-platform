// Publisher: libzmq C API PUB socket; a SUB on loopback decodes every message with depth_wire, and a full send
// high-water mark drops and counts instead of blocking.
#include <gtest/gtest.h>
#include <zmq.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "depthd/config.hpp"
#include "depthd/publisher.hpp"
#include "ros2_common_cpp/depth_wire.hpp"

namespace
{
namespace wire = ros2_common_cpp::depth_wire;

depthd::Transport loopback(int send_hwm = 4)
{
  depthd::Transport t;
  t.bind_address = "127.0.0.1";
  t.pub_port = 0;  // any free port
  t.send_hwm = send_hwm;
  return t;
}

struct Sub
{
  void * ctx;
  void * sock;
  Sub(int port, int rcvhwm = 1000)
  {
    ctx = zmq_ctx_new();
    sock = zmq_socket(ctx, ZMQ_SUB);
    zmq_setsockopt(sock, ZMQ_RCVHWM, &rcvhwm, sizeof(rcvhwm));
    zmq_setsockopt(sock, ZMQ_SUBSCRIBE, "", 0);
    int timeout = 2000;
    zmq_setsockopt(sock, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    const std::string ep = "tcp://127.0.0.1:" + std::to_string(port);
    zmq_connect(sock, ep.c_str());
  }
  ~Sub()
  {
    int linger = 0;
    zmq_setsockopt(sock, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_close(sock);
    zmq_ctx_term(ctx);
  }
  // Receives one multipart message as frames; empty when the timeout expires.
  bool recv(wire::Frames & out)
  {
    std::vector<std::vector<uint8_t>> parts;
    for (;;) {
      zmq_msg_t m;
      zmq_msg_init(&m);
      if (zmq_msg_recv(&m, sock, 0) < 0) {
        zmq_msg_close(&m);
        return false;
      }
      const auto * p = static_cast<const uint8_t *>(zmq_msg_data(&m));
      parts.emplace_back(p, p + zmq_msg_size(&m));
      const bool more = zmq_msg_more(&m) != 0;
      zmq_msg_close(&m);
      if (!more) {
        break;
      }
    }
    if (parts.size() != 3) {
      return false;
    }
    out.topic.assign(parts[0].begin(), parts[0].end());
    out.header = parts[1];
    out.payload = parts[2];
    return true;
  }
};

wire::Header make_header(uint64_t seq, uint32_t w, uint32_t h)
{
  wire::Header header;
  header.sequence = seq;
  header.capture_realtime_ns = 1700000000000000000LL + static_cast<int64_t>(seq);
  header.width = w;
  header.height = h;
  header.frame_id = "stereo_left_optical_frame";
  header.baseline_m = 0.06;
  header.calibrated = true;
  header.pipeline_fps = 12.5F;
  return header;
}

// The SUB connects asynchronously: publish until the first message arrives (a slow joiner loses early ones).
template <typename Send>
wire::Frames first_message(Sub & sub, Send send)
{
  wire::Frames frames;
  for (int i = 0; i < 100; ++i) {
    send();
    int timeout = 50;
    zmq_setsockopt(sub.sock, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    if (sub.recv(frames)) {
      return frames;
    }
  }
  ADD_FAILURE() << "no message received";
  return frames;
}
}  // namespace

TEST(Publisher, BindsAndReportsPort)
{
  depthd::Publisher pub(loopback());
  EXPECT_GT(pub.bound_port(), 0);
  EXPECT_EQ(pub.send_drops(), 0U);
}

TEST(Publisher, DepthRoundTripsThroughDepthWire)
{
  depthd::Publisher pub(loopback());
  Sub sub(pub.bound_port());
  std::vector<uint16_t> depth(8 * 6);
  for (size_t i = 0; i < depth.size(); ++i) {
    depth[i] = static_cast<uint16_t>(i * 37 % 6000);
  }
  const auto frames = first_message(sub, [&] { pub.send_depth(make_header(7, 8, 6), depth); });
  const auto decoded = wire::decode(frames);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(frames.topic, "depth");
  EXPECT_EQ(decoded.header.kind, wire::Kind::kDepth);
  EXPECT_EQ(decoded.header.sequence, 7U);
  EXPECT_EQ(decoded.header.frame_id, "stereo_left_optical_frame");
  EXPECT_EQ(decoded.depth_mm, depth);
}

TEST(Publisher, FusedRoundTripsAndSetsKind)
{
  depthd::Publisher pub(loopback());
  Sub sub(pub.bound_port());
  std::vector<uint16_t> depth(4 * 4, 1234);
  wire::Header header = make_header(9, 4, 4);
  header.kind = wire::Kind::kRgb;  // the publisher owns the kind
  const auto frames = first_message(sub, [&] { pub.send_fused(header, depth); });
  const auto decoded = wire::decode(frames);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(frames.topic, "fused");
  EXPECT_EQ(decoded.header.kind, wire::Kind::kFused);
  EXPECT_EQ(decoded.depth_mm, depth);
}

TEST(Publisher, RgbJpegCarriedUnchanged)
{
  depthd::Publisher pub(loopback());
  Sub sub(pub.bound_port());
  const std::vector<uint8_t> jpeg = {0xFF, 0xD8, 0xFF, 0xE0, 1, 2, 3, 4, 5, 0xFF, 0xD9};
  const auto frames = first_message(sub, [&] { pub.send_rgb(make_header(3, 640, 480), jpeg); });
  const auto decoded = wire::decode(frames);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(frames.topic, "rgb");
  EXPECT_EQ(decoded.header.kind, wire::Kind::kRgb);
  EXPECT_EQ(decoded.jpeg, jpeg);
}

TEST(Publisher, SendReturnsFalseOnInvalidMessageWithoutThrowing)
{
  depthd::Publisher pub(loopback());
  std::vector<uint16_t> wrong(3);
  EXPECT_FALSE(pub.send_depth(make_header(1, 8, 6), wrong));  // width*height != size
  EXPECT_EQ(pub.send_drops(), 0U);                            // an encode error is not a send drop
}

TEST(Publisher, FullHighWaterMarkDropsAndCountsInsteadOfBlocking)
{
  depthd::Publisher pub(loopback(1));
  Sub sub(pub.bound_port(), 1);  // never read: queues fill up
  const std::vector<uint8_t> jpeg(1 << 20, 0xAB);
  // Let the subscription arrive.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto start = std::chrono::steady_clock::now();
  size_t accepted = 0;
  const size_t attempts = 400;
  for (size_t i = 0; i < attempts; ++i) {
    if (pub.send_rgb(make_header(i, 640, 480), jpeg)) {
      ++accepted;
    }
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_GT(pub.send_drops(), 0U);
  EXPECT_EQ(accepted + pub.send_drops(), attempts);
  EXPECT_LT(elapsed, std::chrono::seconds(20)) << "send must never block the pipeline";
}
