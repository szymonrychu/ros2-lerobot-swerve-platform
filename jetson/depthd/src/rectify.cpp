#include "depthd/rectify.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>

namespace depthd
{
namespace
{
void build_maps(const cv::Mat & K, const cv::Mat & D, const cv::Mat & R, const cv::Mat & P, cv::Size size,
                cv::cuda::GpuMat & map_x, cv::cuda::GpuMat & map_y)
{
  cv::Mat mx, my;
  cv::initUndistortRectifyMap(K, D, R, P, size, CV_32FC1, mx, my);
  map_x.upload(mx);
  map_y.upload(my);
}
}  // namespace

Rectification::Rectification(StereoCalibration calibration) : cal_(std::move(calibration))
{
  build_maps(cal_.K1, cal_.D1, cal_.R1, cal_.P1, cal_.size, left_x_, left_y_);
  build_maps(cal_.K2, cal_.D2, cal_.R2, cal_.P2, cal_.size, right_x_, right_y_);
}

void Rectification::rectify(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right,
                            cv::cuda::GpuMat & left_out, cv::cuda::GpuMat & right_out, cv::cuda::Stream & stream) const
{
  cv::cuda::remap(left, left_out, left_x_, left_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), stream);
  cv::cuda::remap(right, right_out, right_x_, right_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), stream);
}

Rectifier::Rectifier(std::string path, double max_rms_px, cv::Size size)
: path_(std::move(path)), max_rms_px_(max_rms_px), size_(size)
{
  reload();
}

bool Rectifier::reload()
{
  std::lock_guard<std::mutex> lock(reload_mutex_);
  auto next = std::make_shared<State>();
  auto load = load_calibration(path_, size_, max_rms_px_);
  if (load.ok()) {
    try {
      next->model = std::make_shared<const Rectification>(std::move(*load.calibration));
    } catch (const cv::Exception & e) {
      next->reason = "cannot build the rectification maps: " + e.err;
    }
  } else {
    next->reason = load.reason;
  }
  const bool ok = next->model != nullptr;
  std::atomic_store(&state_, std::shared_ptr<const State>(std::move(next)));
  return ok;
}

std::shared_ptr<const Rectifier::State> Rectifier::state_ptr() const { return std::atomic_load(&state_); }

std::string Rectifier::reason() const { return state_ptr()->reason; }

std::shared_ptr<const Rectification> Rectifier::snapshot() const { return state_ptr()->model; }

bool Rectifier::rectify(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right, cv::cuda::GpuMat & left_out,
                        cv::cuda::GpuMat & right_out, cv::cuda::Stream & stream) const
{
  const auto model = snapshot();
  if (!model) return false;
  for (const auto * img : {&left, &right})
    if (img->type() != CV_8UC1 || img->size() != model->size()) return false;
  model->rectify(left, right, left_out, right_out, stream);
  return true;
}

}  // namespace depthd
