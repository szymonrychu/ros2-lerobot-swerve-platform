// Port of scipy/optimize/_lsq/trf.py (trf_bounds), common.py and _numdiff.py (2-point, 1-sided at the bounds) as
// used by scipy 1.18 least_squares with the defaults ikpy passes. Function and variable names follow scipy's.
#include "trf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace grasp {
namespace {

constexpr double EPS = std::numeric_limits<double>::epsilon();
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr int MAX_ROWS = TRF_MAX_M + TRF_MAX_N;  // rows of the augmented Jacobian
constexpr int JACOBI_SWEEPS = 80;
constexpr double TRUST_RTOL = 0.01;  // solve_lsq_trust_region rtol
constexpr int TRUST_MAX_ITER = 10;   // solve_lsq_trust_region max_iter

using Mat = std::array<std::array<double, TRF_MAX_N>, TRF_MAX_M>;  // m x n, row-major

double dot(const double *a, const double *b, int k) {
  double s = 0.0;
  for (int i = 0; i < k; ++i) s += a[i] * b[i];
  return s;
}

double norm(const double *a, int k) { return std::sqrt(dot(a, a, k)); }

double norm_inf(const double *a, int k) {
  double r = 0.0;
  for (int i = 0; i < k; ++i) r = std::max(r, std::abs(a[i]));
  return r;
}

// J (m x n) times s (n).
void mat_vec(const Mat &J, int m, int n, const double *s, double *out) {
  for (int i = 0; i < m; ++i) out[i] = dot(J[i].data(), s, n);
}

bool in_bounds(const double *x, const double *lb, const double *ub, int n) {
  for (int i = 0; i < n; ++i)
    if (!(x[i] >= lb[i] && x[i] <= ub[i])) return false;
  return true;
}

// common.make_strictly_feasible (with find_active_constraints).
void make_strictly_feasible(const double *x, const double *lb, const double *ub, int n, double rstep, double *out) {
  for (int i = 0; i < n; ++i) {
    int active = 0;
    if (rstep == 0.0) {
      if (x[i] <= lb[i]) active = -1;
      if (x[i] >= ub[i]) active = 1;
    } else {
      const double lower_dist = x[i] - lb[i];
      const double upper_dist = ub[i] - x[i];
      const double lower_threshold = rstep * std::max(1.0, std::abs(lb[i]));
      const double upper_threshold = rstep * std::max(1.0, std::abs(ub[i]));
      if (std::isfinite(lb[i]) && lower_dist <= std::min(upper_dist, lower_threshold)) active = -1;
      if (std::isfinite(ub[i]) && upper_dist <= std::min(lower_dist, upper_threshold)) active = 1;
    }
    double v = x[i];
    if (active == -1) v = rstep == 0.0 ? std::nextafter(lb[i], ub[i]) : lb[i] + rstep * std::max(1.0, std::abs(lb[i]));
    if (active == 1) v = rstep == 0.0 ? std::nextafter(ub[i], lb[i]) : ub[i] - rstep * std::max(1.0, std::abs(ub[i]));
    if (v < lb[i] || v > ub[i]) v = 0.5 * (lb[i] + ub[i]);
    out[i] = v;
  }
}

// common.CL_scaling_vector.
void cl_scaling_vector(const double *x, const double *g, const double *lb, const double *ub, int n, double *v,
                       double *dv) {
  for (int i = 0; i < n; ++i) {
    v[i] = 1.0;
    dv[i] = 0.0;
    if (g[i] < 0 && std::isfinite(ub[i])) {
      v[i] = ub[i] - x[i];
      dv[i] = -1.0;
    }
    if (g[i] > 0 && std::isfinite(lb[i])) {
      v[i] = x[i] - lb[i];
      dv[i] = 1.0;
    }
  }
}

// _numdiff.approx_derivative(method="2-point") with the bounds: forward differences, flipped or shortened at a bound.
void jacobian(const TrfResidual &fun, int n, int m, const double *x0, const double *f0, const double *lb,
              const double *ub, Mat &J) {
  const double rstep = std::sqrt(EPS);
  TrfVecN x1{};
  TrfVecM f1{};
  for (int i = 0; i < n; ++i) {
    const double sign = x0[i] >= 0 ? 1.0 : -1.0;
    double h = rstep * sign * std::max(1.0, std::abs(x0[i]));
    const double lower_dist = x0[i] - lb[i];
    const double upper_dist = ub[i] - x0[i];
    const double x = x0[i] + h;
    const bool violated = x < lb[i] || x > ub[i];
    const bool fitting = std::abs(h) <= std::max(lower_dist, upper_dist);
    if (violated && fitting) h = -h;
    if (!fitting) h = upper_dist >= lower_dist ? upper_dist : -lower_dist;
    std::copy(x0, x0 + n, x1.begin());
    x1[i] = x0[i] + h;
    fun(x1.data(), f1.data());
    const double dx = (x0[i] + h) - x0[i];
    for (int r = 0; r < m; ++r) J[r][i] = (f1[r] - f0[r]) / dx;
  }
}

// Thin SVD of A (rows x n, column-major columns a[j]) by one-sided Jacobi: singular values s (descending), right
// singular vectors V (V[i][k] = component i of vector k) and uf = U^T f.
void svd_uf(std::array<std::array<double, MAX_ROWS>, TRF_MAX_N> a, int rows, int n, const double *f, double *s,
            std::array<std::array<double, TRF_MAX_N>, TRF_MAX_N> &V, double *uf) {
  std::array<std::array<double, TRF_MAX_N>, TRF_MAX_N> W{};  // W[k] = column k of the accumulated rotation
  for (int k = 0; k < n; ++k) W[k][k] = 1.0;
  for (int sweep = 0; sweep < JACOBI_SWEEPS; ++sweep) {
    bool rotated = false;
    for (int p = 0; p < n - 1; ++p) {
      for (int q = p + 1; q < n; ++q) {
        const double alpha = dot(a[p].data(), a[p].data(), rows);
        const double beta = dot(a[q].data(), a[q].data(), rows);
        const double gamma = dot(a[p].data(), a[q].data(), rows);
        if (gamma == 0.0 || std::abs(gamma) <= EPS * std::sqrt(alpha * beta)) continue;
        rotated = true;
        const double zeta = (beta - alpha) / (2.0 * gamma);
        const double t = (zeta >= 0 ? 1.0 : -1.0) / (std::abs(zeta) + std::sqrt(1.0 + zeta * zeta));
        const double c = 1.0 / std::sqrt(1.0 + t * t);
        const double sn = c * t;
        for (int r = 0; r < rows; ++r) {
          const double ap = a[p][r], aq = a[q][r];
          a[p][r] = c * ap - sn * aq;
          a[q][r] = sn * ap + c * aq;
        }
        for (int r = 0; r < n; ++r) {
          const double wp = W[p][r], wq = W[q][r];
          W[p][r] = c * wp - sn * wq;
          W[q][r] = sn * wp + c * wq;
        }
      }
    }
    if (!rotated) break;
  }
  std::array<int, TRF_MAX_N> order{};
  std::array<double, TRF_MAX_N> sv{};
  for (int k = 0; k < n; ++k) {
    order[k] = k;
    sv[k] = norm(a[k].data(), rows);
  }
  std::stable_sort(order.begin(), order.begin() + n, [&](int i, int j) { return sv[i] > sv[j]; });
  for (int k = 0; k < n; ++k) {
    const int c = order[k];
    s[k] = sv[c];
    uf[k] = sv[c] > 0.0 ? dot(a[c].data(), f, rows) / sv[c] : 0.0;
    for (int r = 0; r < n; ++r) V[r][k] = W[c][r];
  }
}

// common.solve_lsq_trust_region: returns alpha, writes p.
double solve_lsq_trust_region(int n, int m, const double *uf, const double *s,
                              const std::array<std::array<double, TRF_MAX_N>, TRF_MAX_N> &V, double Delta,
                              double initial_alpha, double *p) {
  TrfVecN suf{};
  for (int i = 0; i < n; ++i) suf[i] = s[i] * uf[i];
  auto phi_and_derivative = [&](double alpha, double &phi, double &phi_prime) {
    TrfVecN q{};
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
      const double denom = s[i] * s[i] + alpha;
      q[i] = suf[i] / denom;
      sum += suf[i] * suf[i] / (denom * denom * denom);
    }
    const double p_norm = norm(q.data(), n);
    phi = p_norm - Delta;
    phi_prime = -sum / p_norm;
  };
  bool full_rank = false;
  if (m >= n) {
    const double threshold = EPS * m * s[0];
    full_rank = s[n - 1] > threshold;
  }
  if (full_rank) {
    for (int i = 0; i < n; ++i) {
      double acc = 0.0;
      for (int k = 0; k < n; ++k) acc += V[i][k] * (uf[k] / s[k]);
      p[i] = -acc;
    }
    if (norm(p, n) <= Delta) return 0.0;
  }
  double alpha_upper = norm(suf.data(), n) / Delta;
  double alpha_lower = 0.0;
  if (full_rank) {
    double phi = 0.0, phi_prime = 0.0;
    phi_and_derivative(0.0, phi, phi_prime);
    alpha_lower = -phi / phi_prime;
  }
  double alpha = initial_alpha;
  if (!full_rank && initial_alpha == 0.0) alpha = std::max(0.001 * alpha_upper, std::sqrt(alpha_lower * alpha_upper));
  for (int it = 0; it < TRUST_MAX_ITER; ++it) {
    if (alpha < alpha_lower || alpha > alpha_upper) alpha = std::max(0.001 * alpha_upper, std::sqrt(alpha_lower * alpha_upper));
    double phi = 0.0, phi_prime = 0.0;
    phi_and_derivative(alpha, phi, phi_prime);
    if (phi < 0) alpha_upper = alpha;
    const double ratio = phi / phi_prime;
    alpha_lower = std::max(alpha_lower, alpha - ratio);
    alpha -= (phi + Delta) * ratio / Delta;
    if (std::abs(phi) < TRUST_RTOL * Delta) break;
  }
  for (int i = 0; i < n; ++i) {
    double acc = 0.0;
    for (int k = 0; k < n; ++k) acc += V[i][k] * (suf[k] / (s[k] * s[k] + alpha));
    p[i] = -acc;
  }
  const double scale = Delta / norm(p, n);
  for (int i = 0; i < n; ++i) p[i] *= scale;
  return alpha;
}

// common.evaluate_quadratic for one step s.
double evaluate_quadratic(const Mat &J, int m, int n, const double *g, const double *s, const double *diag) {
  TrfVecM Js{};
  mat_vec(J, m, n, s, Js.data());
  double q = dot(Js.data(), Js.data(), m);
  TrfVecN sd{};
  for (int i = 0; i < n; ++i) sd[i] = s[i] * diag[i];
  q += dot(sd.data(), s, n);
  const double l = dot(s, g, n);
  return 0.5 * q + l;
}

// common.build_quadratic_1d: a, b (and c when s0 is given).
void build_quadratic_1d(const Mat &J, int m, int n, const double *g, const double *s, const double *diag,
                        const double *s0, double &a, double &b, double &c) {
  TrfVecM v{};
  mat_vec(J, m, n, s, v.data());
  a = dot(v.data(), v.data(), m);
  TrfVecN sd{};
  for (int i = 0; i < n; ++i) sd[i] = s[i] * diag[i];
  a += dot(sd.data(), s, n);
  a *= 0.5;
  b = dot(g, s, n);
  c = 0.0;
  if (s0 != nullptr) {
    TrfVecM u{};
    mat_vec(J, m, n, s0, u.data());
    b += dot(u.data(), v.data(), m);
    c = 0.5 * dot(u.data(), u.data(), m) + dot(g, s0, n);
    TrfVecN s0d{};
    for (int i = 0; i < n; ++i) s0d[i] = s0[i] * diag[i];
    b += dot(s0d.data(), s, n);
    c += 0.5 * dot(s0d.data(), s0, n);
  }
}

// common.minimize_quadratic_1d: returns (t, y) of the minimum over [lb, ub].
std::pair<double, double> minimize_quadratic_1d(double a, double b, double lb, double ub, double c) {
  double t[3] = {lb, ub, 0.0};
  int count = 2;
  if (a != 0) {
    const double extremum = -0.5 * b / a;
    if (lb < extremum && extremum < ub) t[count++] = extremum;
  }
  int best = 0;
  double best_y = t[0] * (a * t[0] + b) + c;
  for (int i = 1; i < count; ++i) {
    const double y = t[i] * (a * t[i] + b) + c;
    if (y < best_y) {
      best_y = y;
      best = i;
    }
  }
  return {t[best], best_y};
}

// common.step_size_to_bound: min step and the hits (sign of s on the bounds reached first).
double step_size_to_bound(const double *x, const double *s, const double *lb, const double *ub, int n, int *hits) {
  TrfVecN steps{};
  double min_step = INF;
  for (int i = 0; i < n; ++i) {
    steps[i] = INF;
    if (s[i] != 0.0) steps[i] = std::max((lb[i] - x[i]) / s[i], (ub[i] - x[i]) / s[i]);
    min_step = std::min(min_step, steps[i]);
  }
  if (hits != nullptr)
    for (int i = 0; i < n; ++i) hits[i] = steps[i] == min_step ? (s[i] > 0 ? 1 : (s[i] < 0 ? -1 : 0)) : 0;
  return min_step;
}

// common.intersect_trust_region: the two t with |x + t s| = Delta (t1 <= t2).
std::pair<double, double> intersect_trust_region(const double *x, const double *s, double Delta, int n) {
  const double a = dot(s, s, n);
  if (a == 0) throw std::invalid_argument("`s` is zero.");
  const double b = dot(x, s, n);
  const double c = dot(x, x, n) - Delta * Delta;
  if (c > 0) throw std::invalid_argument("`x` is not within the trust region.");
  const double d = std::sqrt(b * b - a * c);
  const double q = -(b + std::copysign(d, b));
  const double t1 = q / a;
  const double t2 = c / q;
  return t1 < t2 ? std::make_pair(t1, t2) : std::make_pair(t2, t1);
}

// trf.select_step: writes step and step_h, returns the predicted reduction.
double select_step(const double *x, const Mat &J_h, int m, int n, const double *diag_h, const double *g_h,
                   const double *p_in, const double *p_h_in, const double *d, double Delta, const double *lb,
                   const double *ub, double theta, double *step, double *step_h) {
  TrfVecN p{}, p_h{}, xp{};
  std::copy(p_in, p_in + n, p.begin());
  std::copy(p_h_in, p_h_in + n, p_h.begin());
  for (int i = 0; i < n; ++i) xp[i] = x[i] + p[i];
  if (in_bounds(xp.data(), lb, ub, n)) {
    const double p_value = evaluate_quadratic(J_h, m, n, g_h, p_h.data(), diag_h);
    std::copy(p.begin(), p.begin() + n, step);
    std::copy(p_h.begin(), p_h.begin() + n, step_h);
    return -p_value;
  }
  std::array<int, TRF_MAX_N> hits{};
  const double p_stride = step_size_to_bound(x, p.data(), lb, ub, n, hits.data());
  TrfVecN r_h = p_h, r{};
  for (int i = 0; i < n; ++i) {
    if (hits[i] != 0) r_h[i] *= -1;
    r[i] = d[i] * r_h[i];
  }
  TrfVecN x_on_bound{};
  for (int i = 0; i < n; ++i) {
    p[i] *= p_stride;
    p_h[i] *= p_stride;
    x_on_bound[i] = x[i] + p[i];
  }
  const double to_tr_r = intersect_trust_region(p_h.data(), r_h.data(), Delta, n).second;
  const double to_bound_r = step_size_to_bound(x_on_bound.data(), r.data(), lb, ub, n, nullptr);
  double r_stride = std::min(to_bound_r, to_tr_r);
  double r_stride_l = 0.0, r_stride_u = -1.0;
  if (r_stride > 0) {
    r_stride_l = (1 - theta) * p_stride / r_stride;
    r_stride_u = r_stride == to_bound_r ? theta * to_bound_r : to_tr_r;
  }
  double r_value = INF;
  if (r_stride_l <= r_stride_u) {
    double a = 0.0, b = 0.0, c = 0.0;
    build_quadratic_1d(J_h, m, n, g_h, r_h.data(), diag_h, p_h.data(), a, b, c);
    const auto [stride, value] = minimize_quadratic_1d(a, b, r_stride_l, r_stride_u, c);
    r_stride = stride;
    r_value = value;
    for (int i = 0; i < n; ++i) {
      r_h[i] *= r_stride;
      r_h[i] += p_h[i];
      r[i] = r_h[i] * d[i];
    }
  }
  for (int i = 0; i < n; ++i) {
    p[i] *= theta;
    p_h[i] *= theta;
  }
  const double p_value = evaluate_quadratic(J_h, m, n, g_h, p_h.data(), diag_h);
  TrfVecN ag_h{}, ag{};
  for (int i = 0; i < n; ++i) {
    ag_h[i] = -g_h[i];
    ag[i] = d[i] * ag_h[i];
  }
  const double to_tr = Delta / norm(ag_h.data(), n);
  const double to_bound = step_size_to_bound(x, ag.data(), lb, ub, n, nullptr);
  double ag_stride = to_bound < to_tr ? theta * to_bound : to_tr;
  double a = 0.0, b = 0.0, c = 0.0;
  build_quadratic_1d(J_h, m, n, g_h, ag_h.data(), diag_h, nullptr, a, b, c);
  const auto [stride, ag_value] = minimize_quadratic_1d(a, b, 0.0, ag_stride, 0.0);
  ag_stride = stride;
  for (int i = 0; i < n; ++i) {
    ag_h[i] *= ag_stride;
    ag[i] *= ag_stride;
  }
  if (p_value < r_value && p_value < ag_value) {
    std::copy(p.begin(), p.begin() + n, step);
    std::copy(p_h.begin(), p_h.begin() + n, step_h);
    return -p_value;
  }
  if (r_value < p_value && r_value < ag_value) {
    std::copy(r.begin(), r.begin() + n, step);
    std::copy(r_h.begin(), r_h.begin() + n, step_h);
    return -r_value;
  }
  std::copy(ag.begin(), ag.begin() + n, step);
  std::copy(ag_h.begin(), ag_h.begin() + n, step_h);
  return -ag_value;
}

// common.update_tr_radius: returns the new Delta, writes ratio.
double update_tr_radius(double Delta, double actual_reduction, double predicted_reduction, double step_norm,
                        bool bound_hit, double &ratio) {
  if (predicted_reduction > 0)
    ratio = actual_reduction / predicted_reduction;
  else if (predicted_reduction == actual_reduction && actual_reduction == 0)
    ratio = 1;
  else
    ratio = 0;
  if (ratio < 0.25)
    Delta = 0.25 * step_norm;
  else if (ratio > 0.75 && bound_hit)
    Delta *= 2.0;
  return Delta;
}

// common.check_termination: 0 when not terminated.
int check_termination(double dF, double F, double dx_norm, double x_norm, double ratio, double ftol, double xtol) {
  const bool ftol_satisfied = dF < ftol * F && ratio > 0.25;
  const bool xtol_satisfied = dx_norm < xtol * (xtol + x_norm);
  if (ftol_satisfied && xtol_satisfied) return 4;
  if (ftol_satisfied) return 2;
  if (xtol_satisfied) return 3;
  return 0;
}

}  // namespace

TrfResult least_squares_trf(const TrfResidual &fun, int n, int m, const double *x0_in, const double *lb,
                            const double *ub, const TrfOptions &options) {
  if (n < 1 || n > TRF_MAX_N || m < 1 || m > TRF_MAX_M) throw std::invalid_argument("least_squares_trf: bad size");
  if (!in_bounds(x0_in, lb, ub, n)) throw std::invalid_argument("Initial guess is outside of provided bounds");
  TrfVecN x{};
  make_strictly_feasible(x0_in, lb, ub, n, 1e-10, x.data());
  TrfVecM f{};
  fun(x.data(), f.data());
  int nfev = 1;
  Mat J{};
  jacobian(fun, n, m, x.data(), f.data(), lb, ub, J);
  double cost = 0.5 * dot(f.data(), f.data(), m);
  TrfVecN g{};
  for (int j = 0; j < n; ++j) {
    double acc = 0.0;
    for (int i = 0; i < m; ++i) acc += J[i][j] * f[i];
    g[j] = acc;
  }
  TrfVecN v{}, dv{};
  cl_scaling_vector(x.data(), g.data(), lb, ub, n, v.data(), dv.data());
  TrfVecN scaled{};
  for (int i = 0; i < n; ++i) scaled[i] = x[i] / std::sqrt(v[i]);
  double Delta = norm(scaled.data(), n);
  if (Delta == 0) Delta = 1.0;
  const int max_nfev = n * 100;
  double alpha = 0.0;
  int termination_status = 0;
  bool terminated = false;
  double g_norm = 0.0;
  while (true) {
    cl_scaling_vector(x.data(), g.data(), lb, ub, n, v.data(), dv.data());
    for (int i = 0; i < n; ++i) scaled[i] = g[i] * v[i];
    g_norm = norm_inf(scaled.data(), n);
    if (g_norm < options.gtol) {
      termination_status = 1;
      terminated = true;
    }
    if (terminated || nfev == max_nfev) break;
    TrfVecN d{}, diag_h{}, g_h{};
    for (int i = 0; i < n; ++i) {
      d[i] = std::sqrt(v[i]);
      diag_h[i] = g[i] * dv[i];
      g_h[i] = d[i] * g[i];
    }
    Mat J_h{};
    std::array<std::array<double, MAX_ROWS>, TRF_MAX_N> aug{};  // columns of [J * d; diag(diag_h ** 0.5)]
    for (int i = 0; i < m; ++i)
      for (int j = 0; j < n; ++j) {
        J_h[i][j] = J[i][j] * d[j];
        aug[j][i] = J_h[i][j];
      }
    for (int j = 0; j < n; ++j) aug[j][m + j] = std::sqrt(diag_h[j]);
    std::array<double, MAX_ROWS> f_augmented{};
    std::copy(f.begin(), f.begin() + m, f_augmented.begin());
    TrfVecN s{}, uf{};
    std::array<std::array<double, TRF_MAX_N>, TRF_MAX_N> V{};
    svd_uf(aug, m + n, n, f_augmented.data(), s.data(), V, uf.data());
    const double theta = std::max(0.995, 1 - g_norm);
    double actual_reduction = -1;
    TrfVecN x_new{}, step{}, step_h{}, p_h{}, p{}, xs{};
    TrfVecM f_new{};
    double cost_new = 0.0;
    while (actual_reduction <= 0 && nfev < max_nfev) {
      alpha = solve_lsq_trust_region(n, m, uf.data(), s.data(), V, Delta, alpha, p_h.data());
      for (int i = 0; i < n; ++i) p[i] = d[i] * p_h[i];
      const double predicted_reduction = select_step(x.data(), J_h, m, n, diag_h.data(), g_h.data(), p.data(),
                                                     p_h.data(), d.data(), Delta, lb, ub, theta, step.data(),
                                                     step_h.data());
      for (int i = 0; i < n; ++i) xs[i] = x[i] + step[i];
      make_strictly_feasible(xs.data(), lb, ub, n, 0.0, x_new.data());
      fun(x_new.data(), f_new.data());
      ++nfev;
      const double step_h_norm = norm(step_h.data(), n);
      bool finite = true;
      for (int i = 0; i < m; ++i) finite = finite && std::isfinite(f_new[i]);
      if (!finite) {
        Delta = 0.25 * step_h_norm;
        continue;
      }
      cost_new = 0.5 * dot(f_new.data(), f_new.data(), m);
      actual_reduction = cost - cost_new;
      double ratio = 0.0;
      const double Delta_new = update_tr_radius(Delta, actual_reduction, predicted_reduction, step_h_norm,
                                                step_h_norm > 0.95 * Delta, ratio);
      const double step_norm = norm(step.data(), n);
      termination_status = check_termination(actual_reduction, cost, step_norm, norm(x.data(), n), ratio,
                                             options.ftol, options.xtol);
      if (termination_status != 0) {
        terminated = true;
        break;
      }
      alpha *= Delta / Delta_new;
      Delta = Delta_new;
    }
    if (actual_reduction > 0) {
      x = x_new;
      f = f_new;
      cost = cost_new;
      jacobian(fun, n, m, x.data(), f.data(), lb, ub, J);
      for (int j = 0; j < n; ++j) {
        double acc = 0.0;
        for (int i = 0; i < m; ++i) acc += J[i][j] * f[i];
        g[j] = acc;
      }
    }
  }
  TrfResult result;
  result.x = x;
  result.nfev = nfev;
  result.status = termination_status;
  return result;
}

}  // namespace grasp
