// Own binary: linking OpenCV imgcodecs here (as depthd does) would break the other capture tests, which run
// nvjpegenc without going through StereoCapture::start.
#include <gst/gst.h>
#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "depthd/capture.hpp"

using depthd::Config;

// depthd links OpenCV imgcodecs (libjpeg.so.8): once a capture with rgb enabled has started, nvjpegenc in the same
// process must still bind L4T's libnvjpeg, or libjpeg exits the process with "JPEG parameter struct mismatch".
TEST(StereoCaptureJpeg, NvJpegEncodesInAProcessThatLinksImgcodecs)
{
  // Use imgcodecs as depthd does (GET /calibration/capture PNGs), so libjpeg.so.8 is in this process's global scope.
  std::vector<uchar> png;
  ASSERT_TRUE(cv::imencode(".png", cv::Mat(4, 4, CV_8UC1, cv::Scalar(0)), png));
  Config c;
  c.rgb.enabled = true;
  c.capture.left_sensor_id = 7;  // no such sensors: start fails after the plugin is bound, no camera needed
  c.capture.right_sensor_id = 8;
  {
    depthd::StereoCapture cap(c);
    cap.start();
    cap.stop();
  }
  GError * gerr = nullptr;
  GstElement * p = gst_parse_launch(
    "videotestsrc num-buffers=2 ! video/x-raw,width=320,height=240,format=I420 ! nvvidconv ! "
    "video/x-raw(memory:NVMM),format=I420 ! nvjpegenc ! fakesink",
    &gerr);
  ASSERT_NE(p, nullptr) << (gerr ? gerr->message : "parse failed");
  ASSERT_NE(gst_element_set_state(p, GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  GstBus * bus = gst_element_get_bus(p);
  GstMessage * msg = gst_bus_timed_pop_filtered(
    bus, 10 * GST_SECOND, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
  ASSERT_NE(msg, nullptr) << "nvjpegenc pipeline timed out";
  EXPECT_EQ(GST_MESSAGE_TYPE(msg), GST_MESSAGE_EOS);
  gst_message_unref(msg);
  gst_object_unref(bus);
  gst_element_set_state(p, GST_STATE_NULL);
  gst_object_unref(p);
}
