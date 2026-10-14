#include "depthd/scale_align.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace depthd
{
namespace
{
constexpr double kMinMonoGap = 1e-6;  // two-point hypotheses with a smaller mono difference are skipped
constexpr int kRefits = 2;            // least-squares refits on the consensus set

bool is_inlier(double scale, double shift, float m, float y, double tol)
{
  return std::abs(scale * m + shift - y) <= tol * y;
}

int count_inliers(const std::vector<float> & m, const std::vector<float> & y, double scale, double shift, double tol)
{
  int n = 0;
  for (std::size_t i = 0; i < m.size(); ++i) n += is_inlier(scale, shift, m[i], y[i], tol) ? 1 : 0;
  return n;
}

// Least squares y = scale * m + shift over the inliers of (scale, shift); false when degenerate.
bool refit(const std::vector<float> & m, const std::vector<float> & y, double tol, double & scale, double & shift)
{
  double sm = 0, sy = 0, smm = 0, smy = 0;
  std::size_t n = 0;
  for (std::size_t i = 0; i < m.size(); ++i) {
    if (!is_inlier(scale, shift, m[i], y[i], tol)) continue;
    sm += m[i];
    sy += y[i];
    smm += static_cast<double>(m[i]) * m[i];
    smy += static_cast<double>(m[i]) * y[i];
    ++n;
  }
  if (n < 2) return false;
  const double nn = static_cast<double>(n);
  const double var = smm - sm * sm / nn;
  if (var <= 0.0) return false;
  scale = (smy - sm * sy / nn) / var;
  shift = (sy - scale * sm) / nn;
  return true;
}

double quantile(std::vector<float> v, double q)
{
  const std::size_t k = static_cast<std::size_t>(q * static_cast<double>(v.size() - 1));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}
}  // namespace

ScaleShift fit_scale_shift(const std::vector<float> & mono, const std::vector<float> & target, const AlignParams & p)
{
  if (mono.size() != target.size()) throw std::invalid_argument("fit_scale_shift: mono and target differ in size");
  std::vector<float> m;
  std::vector<float> y;
  m.reserve(mono.size());
  y.reserve(mono.size());
  for (std::size_t i = 0; i < mono.size(); ++i) {
    if (std::isfinite(mono[i]) && std::isfinite(target[i]) && target[i] > 0.0F) {
      m.push_back(mono[i]);
      y.push_back(target[i]);
    }
  }
  ScaleShift r;
  r.samples = static_cast<int>(m.size());
  if (r.samples < p.min_inliers) {
    r.reason = "too few valid stereo pixels: " + std::to_string(r.samples) + " < " + std::to_string(p.min_inliers);
    return r;
  }
  const double median = quantile(y, 0.5);
  if ((quantile(y, 0.9) - quantile(y, 0.1)) < p.min_y_spread * median) {
    r.reason = "stereo depth has no spread to scale against";
    return r;
  }

  std::mt19937 rng(p.seed);
  std::uniform_int_distribution<std::size_t> pick(0, m.size() - 1);
  double best_scale = 0.0, best_shift = 0.0;
  int best = -1;
  for (int it = 0; it < p.iterations; ++it) {
    const std::size_t a = pick(rng);
    const std::size_t b = pick(rng);
    const double dm = static_cast<double>(m[a]) - m[b];
    if (std::abs(dm) < kMinMonoGap) continue;
    const double scale = (static_cast<double>(y[a]) - y[b]) / dm;
    if (!(scale > 0.0)) continue;
    const double shift = y[a] - scale * m[a];
    const int n = count_inliers(m, y, scale, shift, p.inlier_rel_tol);
    if (n > best) {
      best = n;
      best_scale = scale;
      best_shift = shift;
    }
  }
  if (best < 0) {
    r.reason = "no positive-scale hypothesis";
    return r;
  }
  for (int i = 0; i < kRefits; ++i) {
    double s = best_scale, t = best_shift;
    if (!refit(m, y, p.inlier_rel_tol, s, t) || !(s > 0.0)) break;
    best_scale = s;
    best_shift = t;
  }
  r.inliers = count_inliers(m, y, best_scale, best_shift, p.inlier_rel_tol);
  r.scale = best_scale;
  r.shift = best_shift;
  if (r.inliers < p.min_inliers) {
    r.reason = "too few inliers: " + std::to_string(r.inliers) + " < " + std::to_string(p.min_inliers);
    return r;
  }
  r.ok = true;
  return r;
}

FillResult align_and_fill(
  const cv::Mat & mono_inv, const cv::Mat & stereo_mm, const AlignParams & p, std::vector<uint16_t> & fused)
{
  if (mono_inv.type() != CV_32FC1 || stereo_mm.type() != CV_16UC1 || mono_inv.size() != stereo_mm.size()) {
    throw std::invalid_argument("align_and_fill: need CV_32FC1 mono and CV_16UC1 stereo of the same size");
  }
  FillResult r;
  std::size_t valid = 0;
  for (int y = 0; y < stereo_mm.rows; ++y) {
    const uint16_t * s = stereo_mm.ptr<uint16_t>(y);
    for (int x = 0; x < stereo_mm.cols; ++x) valid += s[x] ? 1 : 0;
  }
  r.holes = stereo_mm.total() - valid;

  // Evenly strided valid pixels for the fit.
  const std::size_t stride = std::max<std::size_t>(1, (valid + p.max_samples - 1) / std::max(1, p.max_samples));
  std::vector<float> m;
  std::vector<float> t;
  m.reserve(valid / stride + 1);
  t.reserve(valid / stride + 1);
  std::size_t k = 0;
  for (int y = 0; y < stereo_mm.rows; ++y) {
    const uint16_t * s = stereo_mm.ptr<uint16_t>(y);
    const float * mo = mono_inv.ptr<float>(y);
    for (int x = 0; x < stereo_mm.cols; ++x) {
      if (!s[x] || (k++ % stride) != 0) continue;
      m.push_back(mo[x]);
      t.push_back(1000.0F / static_cast<float>(s[x]));
    }
  }
  r.fit = fit_scale_shift(m, t, p);
  if (!r.fit.ok) return r;

  const double min_mm = p.min_depth_m * 1000.0;
  const double max_mm = p.max_depth_m * 1000.0;
  fused.resize(stereo_mm.total());
  for (int y = 0; y < stereo_mm.rows; ++y) {
    const uint16_t * s = stereo_mm.ptr<uint16_t>(y);
    const float * mo = mono_inv.ptr<float>(y);
    uint16_t * out = fused.data() + static_cast<std::size_t>(y) * stereo_mm.cols;
    for (int x = 0; x < stereo_mm.cols; ++x) {
      if (s[x]) {
        out[x] = s[x];
        continue;
      }
      out[x] = 0;
      const double inv = r.fit.scale * mo[x] + r.fit.shift;  // 1/m
      if (!std::isfinite(inv) || inv <= 0.0) continue;
      const double mm = 1000.0 / inv;
      if (mm < min_mm || mm > max_mm || mm > 65535.0) continue;
      out[x] = static_cast<uint16_t>(std::lround(mm));
      ++r.filled;
    }
  }
  return r;
}

}  // namespace depthd
