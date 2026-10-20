#include "ground_estimator_cpp/colour.hpp"

#include <cmath>
#include <cstring>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "ground_estimator_cpp/segmentation.hpp"

namespace ground_estimator_cpp
{

namespace
{

constexpr double RGB_SCALE = 255.0;
constexpr double COVARIANCE_RIDGE = 1.0;  // Lab units squared; keeps the covariance invertible on a flat colour
constexpr uint32_t POINT_STEP = 17;       // x, y, z, rgb float32 and label uint8, unpadded
constexpr uint32_t OFFSET_RGB = 12;
constexpr uint32_t OFFSET_LABEL = 16;

Eigen::Vector3d lab_vector(const cv::Vec3b & rgb)
{
  const cv::Vec3f lab = rgb_to_lab(rgb);
  return Eigen::Vector3d(lab[0], lab[1], lab[2]);
}

}  // namespace

cv::Vec3f rgb_to_lab(const cv::Vec3b & rgb)
{
  const cv::Mat src(1, 1, CV_32FC3, cv::Scalar(rgb[0] / RGB_SCALE, rgb[1] / RGB_SCALE, rgb[2] / RGB_SCALE));
  cv::Mat dst;
  cv::cvtColor(src, dst, cv::COLOR_RGB2Lab);
  return dst.at<cv::Vec3f>(0, 0);
}

void FloorColourModel::update(const std::vector<cv::Vec3b> & rgb_floor_samples)
{
  const double forgetting = colour_.forgetting;
  if (rgb_floor_samples.empty()) {
    return;
  }
  const double n = static_cast<double>(rgb_floor_samples.size());
  Eigen::Vector3d batch_mean = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> labs;
  labs.reserve(rgb_floor_samples.size());
  for (const auto & rgb : rgb_floor_samples) {
    labs.push_back(lab_vector(rgb));
    batch_mean += labs.back();
  }
  batch_mean /= n;
  Eigen::Matrix3d batch_cov = Eigen::Matrix3d::Zero();
  for (const auto & lab : labs) {
    batch_cov += (lab - batch_mean) * (lab - batch_mean).transpose();
  }
  batch_cov /= n;

  if (count_ == 0) {
    mean_ = batch_mean;
    cov_ = batch_cov;
  } else {
    const Eigen::Vector3d shift = batch_mean - mean_;
    mean_ += forgetting * shift;
    // Exponentially weighted covariance: the shift term keeps the model wide while it follows a colour change.
    cov_ =
      (1.0 - forgetting) * cov_ + forgetting * batch_cov + forgetting * (1.0 - forgetting) * shift * shift.transpose();
  }
  count_ += rgb_floor_samples.size();
  cov_inv_ = (cov_ + COVARIANCE_RIDGE * Eigen::Matrix3d::Identity()).inverse();
  ready_ = count_ >= static_cast<std::size_t>(colour_.min_floor_samples);
}

double FloorColourModel::mahalanobis(const cv::Vec3b & rgb) const
{
  if (count_ == 0) {
    return 0.0;
  }
  const Eigen::Vector3d d = lab_vector(rgb) - mean_;
  return std::sqrt(std::max(0.0, d.dot(cov_inv_ * d)));
}

LabelStats label_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud_base, const Plane & plane, const BandsConfig & bands,
  const ColourConfig & colour, double floor_distance_m, FloorColourModel & model, std::vector<Label> & labels)
{
  LabelStats stats;
  labels.assign(cloud_base.size(), Label::unlabelled);
  std::vector<std::size_t> floor_index;
  std::vector<cv::Vec3b> floor_rgb;
  for (std::size_t i = 0; i < cloud_base.size(); ++i) {
    const auto & p = cloud_base[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const Eigen::Vector3d v(p.x, p.y, p.z);
    switch (classify(height_above(plane, v), std::hypot(v.x(), v.y()), bands, floor_distance_m)) {
      case Band::floor:
        labels[i] = Label::floor;
        floor_index.push_back(i);
        floor_rgb.emplace_back(p.r, p.g, p.b);
        break;
      case Band::above:
        labels[i] = Label::above_floor;
        break;
      case Band::below:
        labels[i] = Label::below_floor;
        break;
      case Band::none:
        break;
    }
  }
  if (colour.enabled) {
    model.update(floor_rgb);
    if (model.ready()) {
      for (std::size_t k = 0; k < floor_index.size(); ++k) {
        if (model.mahalanobis(floor_rgb[k]) > colour.max_mahalanobis) {
          labels[floor_index[k]] = Label::floor_colour_mismatch;
        }
      }
    }
  }
  for (const Label l : labels) {
    switch (l) {
      case Label::floor:
        ++stats.floor;
        break;
      case Label::above_floor:
        ++stats.above;
        break;
      case Label::below_floor:
        ++stats.below;
        break;
      case Label::floor_colour_mismatch:
        ++stats.mismatch;
        break;
      case Label::unlabelled:
        ++stats.unlabelled;
        break;
    }
  }
  if (stats.floor + stats.mismatch > 0) {
    stats.mismatch_ratio = static_cast<double>(stats.mismatch) / static_cast<double>(stats.floor + stats.mismatch);
  }
  return stats;
}

std::vector<MismatchPoint> mismatch_points(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud_base, const std::vector<Label> & labels, const Plane & plane)
{
  std::vector<MismatchPoint> out;
  for (std::size_t i = 0; i < cloud_base.size() && i < labels.size(); ++i) {
    if (labels[i] == Label::floor_colour_mismatch) {
      const Eigen::Vector3d v(cloud_base[i].x, cloud_base[i].y, cloud_base[i].z);
      out.push_back({v, height_above(plane, v)});
    }
  }
  return out;
}

sensor_msgs::msg::PointCloud2 to_labelled_msg(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const std::vector<Label> & labels, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  const struct
  {
    const char * name;
    uint32_t offset;
    uint8_t type;
  } layout[] = {
    {"x", 0, PointField::FLOAT32},
    {"y", 4, PointField::FLOAT32},
    {"z", 8, PointField::FLOAT32},
    {"rgb", OFFSET_RGB, PointField::FLOAT32},
    {"label", OFFSET_LABEL, PointField::UINT8}};
  for (const auto & f : layout) {
    PointField field;
    field.name = f.name;
    field.offset = f.offset;
    field.datatype = f.type;
    field.count = 1;
    msg.fields.push_back(field);
  }
  msg.is_bigendian = false;
  msg.point_step = POINT_STEP;
  msg.row_step = POINT_STEP * msg.width;
  msg.data.resize(static_cast<std::size_t>(msg.row_step));
  bool dense = true;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    dense = dense && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    uint8_t * out = &msg.data[i * POINT_STEP];
    const float xyz[3] = {p.x, p.y, p.z};
    std::memcpy(out, xyz, sizeof(xyz));
    const uint32_t packed = (static_cast<uint32_t>(p.r) << 16) | (static_cast<uint32_t>(p.g) << 8) | p.b;
    std::memcpy(out + OFFSET_RGB, &packed, sizeof(packed));
    out[OFFSET_LABEL] = static_cast<uint8_t>(i < labels.size() ? labels[i] : Label::unlabelled);
  }
  msg.is_dense = dense;
  return msg;
}

}  // namespace ground_estimator_cpp
