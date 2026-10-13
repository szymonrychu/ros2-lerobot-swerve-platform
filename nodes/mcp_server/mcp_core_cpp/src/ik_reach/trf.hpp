// scipy.optimize.least_squares(method='trf', jac='2-point', bounds=(lb, ub)) with the scipy 1.18 defaults ikpy uses
// (ftol = xtol = gtol = 1e-8, x_scale 1, linear loss, tr_solver 'exact', max_nfev 100 * n), ported step for step from
// scipy/optimize/_lsq/trf.py (trf_bounds, select_step), _lsq/common.py and _numdiff.py (the bounded 2-point Jacobian),
// so the iterates follow scipy's up to floating point rounding. Fixed sizes: N variables, M residuals.
#pragma once

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ik_reach::trf {

constexpr double EPS = DBL_EPSILON;
constexpr double TOL = 1e-8;  // ftol, xtol, gtol
constexpr double INF = std::numeric_limits<double>::infinity();

template <int N>
using VecN = std::array<double, N>;

template <int N>
double dot(const VecN<N> &a, const VecN<N> &b) {
  double s = 0.0;
  for (int i = 0; i < N; ++i) s += a[i] * b[i];
  return s;
}

template <int N>
double norm(const VecN<N> &a) {
  return std::sqrt(dot<N>(a, a));
}

template <int N>
bool in_bounds(const VecN<N> &x, const VecN<N> &lb, const VecN<N> &ub) {
  for (int i = 0; i < N; ++i)
    if (!(x[i] >= lb[i] && x[i] <= ub[i])) return false;
  return true;
}

// find_active_constraints + make_strictly_feasible (common.py).
template <int N>
VecN<N> make_strictly_feasible(const VecN<N> &x, const VecN<N> &lb, const VecN<N> &ub, double rstep) {
  VecN<N> out = x;
  for (int i = 0; i < N; ++i) {
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
    if (rstep == 0.0) {
      if (active == -1) out[i] = std::nextafter(lb[i], ub[i]);
      if (active == 1) out[i] = std::nextafter(ub[i], lb[i]);
    } else {
      if (active == -1) out[i] = lb[i] + rstep * std::max(1.0, std::abs(lb[i]));
      if (active == 1) out[i] = ub[i] - rstep * std::max(1.0, std::abs(ub[i]));
    }
    if (out[i] < lb[i] || out[i] > ub[i]) out[i] = 0.5 * (lb[i] + ub[i]);
  }
  return out;
}

// Dense M x N matrix, row major.
template <int M, int N>
using MatMN = std::array<std::array<double, N>, M>;

// approx_derivative(fun, x, method='2-point', f0=f, bounds=(lb, ub)) with the default relative step.
template <int M, int N, class Fun>
MatMN<M, N> jacobian(Fun &fun, const VecN<N> &x, const VecN<M> &f, const VecN<N> &lb, const VecN<N> &ub) {
  const double rstep = std::sqrt(EPS);
  MatMN<M, N> jac{};
  for (int i = 0; i < N; ++i) {
    const double sign = x[i] >= 0.0 ? 1.0 : -1.0;
    double h = rstep * sign * std::max(1.0, std::abs(x[i]));
    // _adjust_scheme_to_bounds(x0, h, 1, '1-sided', lb, ub)
    const double lower_dist = x[i] - lb[i];
    const double upper_dist = ub[i] - x[i];
    const double moved = x[i] + h;
    const bool violated = moved < lb[i] || moved > ub[i];
    const bool fitting = std::abs(h) <= std::max(lower_dist, upper_dist);
    if (violated && fitting) h = -h;
    if (!fitting) h = upper_dist >= lower_dist ? upper_dist : -lower_dist;
    VecN<N> x1 = x;
    x1[i] = x[i] + h;
    const double dx = (x[i] + h) - x[i];
    VecN<M> f1{};
    fun(x1, f1);
    for (int r = 0; r < M; ++r) jac[r][i] = (f1[r] - f[r]) / dx;
  }
  return jac;
}

// Thin SVD of an R x N matrix (R >= N) by one-sided Jacobi rotations: a = U diag(s) V^T, s descending.
template <int R, int N>
void svd(const MatMN<R, N> &a, MatMN<R, N> &u, VecN<N> &s, MatMN<N, N> &v) {
  u = a;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j) v[i][j] = i == j ? 1.0 : 0.0;
  for (int sweep = 0; sweep < 80; ++sweep) {
    bool rotated = false;
    for (int p = 0; p < N - 1; ++p) {
      for (int q = p + 1; q < N; ++q) {
        double alpha = 0.0, beta = 0.0, gamma = 0.0;
        for (int i = 0; i < R; ++i) {
          alpha += u[i][p] * u[i][p];
          beta += u[i][q] * u[i][q];
          gamma += u[i][p] * u[i][q];
        }
        if (gamma == 0.0 || std::abs(gamma) <= EPS * std::sqrt(alpha * beta)) continue;
        rotated = true;
        const double zeta = (beta - alpha) / (2.0 * gamma);
        const double t = (zeta >= 0.0 ? 1.0 : -1.0) / (std::abs(zeta) + std::sqrt(1.0 + zeta * zeta));
        const double c = 1.0 / std::sqrt(1.0 + t * t);
        const double sn = c * t;
        for (int i = 0; i < R; ++i) {
          const double up = u[i][p], uq = u[i][q];
          u[i][p] = c * up - sn * uq;
          u[i][q] = sn * up + c * uq;
        }
        for (int i = 0; i < N; ++i) {
          const double vp = v[i][p], vq = v[i][q];
          v[i][p] = c * vp - sn * vq;
          v[i][q] = sn * vp + c * vq;
        }
      }
    }
    if (!rotated) break;
  }
  std::array<int, N> idx{};
  VecN<N> norms{};
  for (int j = 0; j < N; ++j) {
    idx[j] = j;
    double sq = 0.0;
    for (int i = 0; i < R; ++i) sq += u[i][j] * u[i][j];
    norms[j] = std::sqrt(sq);
  }
  std::stable_sort(idx.begin(), idx.end(), [&](int a_, int b_) { return norms[a_] > norms[b_]; });
  const MatMN<R, N> u0 = u;
  const MatMN<N, N> v0 = v;
  for (int k = 0; k < N; ++k) {
    const int j = idx[k];
    s[k] = norms[j];
    for (int i = 0; i < R; ++i) u[i][k] = norms[j] > 0.0 ? u0[i][j] / norms[j] : 0.0;
    for (int i = 0; i < N; ++i) v[i][k] = v0[i][j];
  }
}

// step_size_to_bound: smallest step along s to a bound and which bounds it hits (sign of s there, else 0).
template <int N>
double step_size_to_bound(const VecN<N> &x, const VecN<N> &s, const VecN<N> &lb, const VecN<N> &ub,
                          std::array<int, N> &hits) {
  VecN<N> steps{};
  double min_step = INF;
  for (int i = 0; i < N; ++i) {
    steps[i] = INF;
    if (s[i] != 0.0) steps[i] = std::max((lb[i] - x[i]) / s[i], (ub[i] - x[i]) / s[i]);
    min_step = std::min(min_step, steps[i]);
  }
  for (int i = 0; i < N; ++i) hits[i] = steps[i] == min_step ? (s[i] > 0.0 ? 1 : (s[i] < 0.0 ? -1 : 0)) : 0;
  return min_step;
}

// intersect_trust_region: the two step sizes along s from x to the trust region boundary.
template <int N>
std::pair<double, double> intersect_trust_region(const VecN<N> &x, const VecN<N> &s, double delta) {
  const double a = dot<N>(s, s);
  if (a == 0.0) throw std::domain_error("`s` is zero.");
  const double b = dot<N>(x, s);
  const double c = dot<N>(x, x) - delta * delta;
  if (c > 0.0) throw std::domain_error("`x` is not within the trust region.");
  const double d = std::sqrt(b * b - a * c);
  const double q = -(b + std::copysign(d, b));
  const double t1 = q / a;
  const double t2 = c / q;
  return t1 < t2 ? std::make_pair(t1, t2) : std::make_pair(t2, t1);
}

template <int M, int N>
VecN<M> mat_vec(const MatMN<M, N> &j, const VecN<N> &s) {
  VecN<M> out{};
  for (int r = 0; r < M; ++r) {
    double acc = 0.0;
    for (int c = 0; c < N; ++c) acc += j[r][c] * s[c];
    out[r] = acc;
  }
  return out;
}

// evaluate_quadratic(J, g, s, diag): 0.5 s^T (J^T J + diag) s + g^T s.
template <int M, int N>
double evaluate_quadratic(const MatMN<M, N> &j, const VecN<N> &g, const VecN<N> &s, const VecN<N> &diag) {
  const VecN<M> js = mat_vec<M, N>(j, s);
  double q = dot<M>(js, js);
  VecN<N> sd{};
  for (int i = 0; i < N; ++i) sd[i] = s[i] * diag[i];
  q += dot<N>(sd, s);
  return 0.5 * q + dot<N>(s, g);
}

struct Quadratic {
  double a, b, c;
};

// build_quadratic_1d(J, g, s, diag, s0): coefficients of f(t) = a t^2 + b t + c along s from s0.
template <int M, int N>
Quadratic build_quadratic_1d(const MatMN<M, N> &j, const VecN<N> &g, const VecN<N> &s, const VecN<N> &diag,
                             const VecN<N> *s0) {
  const VecN<M> v = mat_vec<M, N>(j, s);
  double a = dot<M>(v, v);
  VecN<N> sd{};
  for (int i = 0; i < N; ++i) sd[i] = s[i] * diag[i];
  a += dot<N>(sd, s);
  a *= 0.5;
  double b = dot<N>(g, s);
  double c = 0.0;
  if (s0 != nullptr) {
    const VecN<M> u = mat_vec<M, N>(j, *s0);
    b += dot<M>(u, v);
    c = 0.5 * dot<M>(u, u) + dot<N>(g, *s0);
    VecN<N> s0d{};
    for (int i = 0; i < N; ++i) s0d[i] = (*s0)[i] * diag[i];
    b += dot<N>(s0d, s);
    c += 0.5 * dot<N>(s0d, *s0);
  }
  return {a, b, c};
}

// minimize_quadratic_1d: argmin and minimum of a t^2 + b t + c on [lb, ub].
inline std::pair<double, double> minimize_quadratic_1d(double a, double b, double lb, double ub, double c) {
  double t[3] = {lb, ub, 0.0};
  int n = 2;
  if (a != 0.0) {
    const double extremum = -0.5 * b / a;
    if (lb < extremum && extremum < ub) t[n++] = extremum;
  }
  int best = 0;
  double best_y = t[0] * (a * t[0] + b) + c;
  for (int i = 1; i < n; ++i) {
    const double y = t[i] * (a * t[i] + b) + c;
    if (y < best_y) {
      best = i;
      best_y = y;
    }
  }
  return {t[best], best_y};
}

// solve_lsq_trust_region (common.py): the regularized least-squares step inside the trust region.
template <int N>
VecN<N> solve_lsq_trust_region(int m, const VecN<N> &uf, const VecN<N> &s, const MatMN<N, N> &v, double delta,
                               double &alpha) {
  const double rtol = 0.01;
  const int max_iter = 10;
  VecN<N> suf{};
  for (int i = 0; i < N; ++i) suf[i] = s[i] * uf[i];
  bool full_rank = false;
  if (m >= N) full_rank = s[N - 1] > EPS * m * s[0];
  auto v_dot = [&](const VecN<N> &w) {
    VecN<N> out{};
    for (int r = 0; r < N; ++r) {
      double acc = 0.0;
      for (int c = 0; c < N; ++c) acc += v[r][c] * w[c];
      out[r] = acc;
    }
    return out;
  };
  auto phi_and_derivative = [&](double a_, double &phi, double &phi_prime) {
    VecN<N> ratio{};
    double sum = 0.0;
    for (int i = 0; i < N; ++i) {
      const double denom = s[i] * s[i] + a_;
      ratio[i] = suf[i] / denom;
      sum += suf[i] * suf[i] / std::pow(denom, 3.0);
    }
    const double p_norm = norm<N>(ratio);
    phi = p_norm - delta;
    phi_prime = -sum / p_norm;
  };
  if (full_rank) {
    VecN<N> w{};
    for (int i = 0; i < N; ++i) w[i] = uf[i] / s[i];
    VecN<N> p = v_dot(w);
    for (int i = 0; i < N; ++i) p[i] = -p[i];
    if (norm<N>(p) <= delta) {
      alpha = 0.0;
      return p;
    }
  }
  double alpha_upper = norm<N>(suf) / delta;
  double alpha_lower = 0.0;
  if (full_rank) {
    double phi, phi_prime;
    phi_and_derivative(0.0, phi, phi_prime);
    alpha_lower = -phi / phi_prime;
  }
  double a_ = alpha;
  if (!full_rank && alpha == 0.0) a_ = std::max(0.001 * alpha_upper, std::pow(alpha_lower * alpha_upper, 0.5));
  for (int it = 0; it < max_iter; ++it) {
    if (a_ < alpha_lower || a_ > alpha_upper) a_ = std::max(0.001 * alpha_upper, std::pow(alpha_lower * alpha_upper, 0.5));
    double phi, phi_prime;
    phi_and_derivative(a_, phi, phi_prime);
    if (phi < 0.0) alpha_upper = a_;
    const double ratio = phi / phi_prime;
    alpha_lower = std::max(alpha_lower, a_ - ratio);
    a_ -= (phi + delta) * ratio / delta;
    if (std::abs(phi) < rtol * delta) break;
  }
  VecN<N> w{};
  for (int i = 0; i < N; ++i) w[i] = suf[i] / (s[i] * s[i] + a_);
  VecN<N> p = v_dot(w);
  for (int i = 0; i < N; ++i) p[i] = -p[i];
  const double scale = delta / norm<N>(p);
  for (int i = 0; i < N; ++i) p[i] *= scale;
  alpha = a_;
  return p;
}

// select_step (trf.py): trust-region step, reflected step or anti-gradient step, whichever is best.
template <int M, int N>
double select_step(const VecN<N> &x, const MatMN<M, N> &j_h, const VecN<N> &diag_h, const VecN<N> &g_h, VecN<N> p,
                   VecN<N> p_h, const VecN<N> &d, double delta, const VecN<N> &lb, const VecN<N> &ub, double theta,
                   VecN<N> &step, VecN<N> &step_h) {
  VecN<N> moved{};
  for (int i = 0; i < N; ++i) moved[i] = x[i] + p[i];
  if (in_bounds<N>(moved, lb, ub)) {
    step = p;
    step_h = p_h;
    return -evaluate_quadratic<M, N>(j_h, g_h, p_h, diag_h);
  }
  std::array<int, N> hits{};
  const double p_stride = step_size_to_bound<N>(x, p, lb, ub, hits);
  VecN<N> r_h = p_h;
  for (int i = 0; i < N; ++i)
    if (hits[i] != 0) r_h[i] *= -1.0;
  VecN<N> r{};
  for (int i = 0; i < N; ++i) r[i] = d[i] * r_h[i];
  for (int i = 0; i < N; ++i) {
    p[i] *= p_stride;
    p_h[i] *= p_stride;
  }
  VecN<N> x_on_bound{};
  for (int i = 0; i < N; ++i) x_on_bound[i] = x[i] + p[i];
  const double to_tr = intersect_trust_region<N>(p_h, r_h, delta).second;
  std::array<int, N> unused{};
  const double to_bound = step_size_to_bound<N>(x_on_bound, r, lb, ub, unused);
  double r_stride = std::min(to_bound, to_tr);
  double r_stride_l, r_stride_u;
  if (r_stride > 0.0) {
    r_stride_l = (1.0 - theta) * p_stride / r_stride;
    r_stride_u = r_stride == to_bound ? theta * to_bound : to_tr;
  } else {
    r_stride_l = 0.0;
    r_stride_u = -1.0;
  }
  double r_value;
  if (r_stride_l <= r_stride_u) {
    const Quadratic q = build_quadratic_1d<M, N>(j_h, g_h, r_h, diag_h, &p_h);
    const auto [stride, value] = minimize_quadratic_1d(q.a, q.b, r_stride_l, r_stride_u, q.c);
    r_stride = stride;
    r_value = value;
    for (int i = 0; i < N; ++i) {
      r_h[i] *= r_stride;
      r_h[i] += p_h[i];
      r[i] = r_h[i] * d[i];
    }
  } else {
    r_value = INF;
  }
  for (int i = 0; i < N; ++i) {
    p[i] *= theta;
    p_h[i] *= theta;
  }
  const double p_value = evaluate_quadratic<M, N>(j_h, g_h, p_h, diag_h);
  VecN<N> ag_h{}, ag{};
  for (int i = 0; i < N; ++i) {
    ag_h[i] = -g_h[i];
    ag[i] = d[i] * ag_h[i];
  }
  const double to_tr_ag = delta / norm<N>(ag_h);
  const double to_bound_ag = step_size_to_bound<N>(x, ag, lb, ub, unused);
  double ag_stride = to_bound_ag < to_tr_ag ? theta * to_bound_ag : to_tr_ag;
  const Quadratic qa = build_quadratic_1d<M, N>(j_h, g_h, ag_h, diag_h, nullptr);
  const auto [ag_s, ag_value] = minimize_quadratic_1d(qa.a, qa.b, 0.0, ag_stride, 0.0);
  ag_stride = ag_s;
  for (int i = 0; i < N; ++i) {
    ag_h[i] *= ag_stride;
    ag[i] *= ag_stride;
  }
  if (p_value < r_value && p_value < ag_value) {
    step = p;
    step_h = p_h;
    return -p_value;
  }
  if (r_value < p_value && r_value < ag_value) {
    step = r;
    step_h = r_h;
    return -r_value;
  }
  step = ag;
  step_h = ag_h;
  return -ag_value;
}

// least_squares(fun, x0, bounds=(lb, ub)) as ikpy calls it; fun(x, f) writes the M residuals. Returns res.x.
template <int M, int N, class Fun>
VecN<N> least_squares(Fun &&fun, const VecN<N> &x_start, const VecN<N> &lb, const VecN<N> &ub) {
  if (!in_bounds<N>(x_start, lb, ub)) throw std::domain_error("Initial guess is outside of provided bounds");
  const VecN<N> x0 = make_strictly_feasible<N>(x_start, lb, ub, 1e-10);
  VecN<N> x = x0;
  VecN<M> f{};
  fun(x, f);
  for (int r = 0; r < M; ++r)
    if (!std::isfinite(f[r])) throw std::domain_error("Residuals are not finite in the initial point.");
  MatMN<M, N> jac = jacobian<M, N>(fun, x, f, lb, ub);
  int nfev = 1;
  const int max_nfev = 100 * N;
  double cost = 0.5 * dot<M>(f, f);
  auto grad = [&]() {
    VecN<N> g{};
    for (int c = 0; c < N; ++c) {
      double acc = 0.0;
      for (int r = 0; r < M; ++r) acc += jac[r][c] * f[r];
      g[c] = acc;
    }
    return g;
  };
  VecN<N> g = grad();
  auto cl_scaling = [&](VecN<N> &v, VecN<N> &dv) {
    for (int i = 0; i < N; ++i) {
      v[i] = 1.0;
      dv[i] = 0.0;
      if (g[i] < 0.0 && std::isfinite(ub[i])) {
        v[i] = ub[i] - x[i];
        dv[i] = -1.0;
      }
      if (g[i] > 0.0 && std::isfinite(lb[i])) {
        v[i] = x[i] - lb[i];
        dv[i] = 1.0;
      }
    }
  };
  VecN<N> v{}, dv{};
  cl_scaling(v, dv);
  VecN<N> scaled{};
  for (int i = 0; i < N; ++i) scaled[i] = x0[i] * 1.0 / std::sqrt(v[i]);  // numpy array ** 0.5 is sqrt
  double delta = norm<N>(scaled);
  if (delta == 0.0) delta = 1.0;
  constexpr int R = M + N;
  VecN<R> f_aug{};
  double alpha = 0.0;
  bool done = false;
  while (true) {
    cl_scaling(v, dv);
    double g_norm = 0.0;
    for (int i = 0; i < N; ++i) g_norm = std::max(g_norm, std::abs(g[i] * v[i]));
    if (g_norm < TOL) done = true;
    if (done || nfev == max_nfev) break;
    VecN<N> d{}, diag_h{}, g_h{};
    for (int i = 0; i < N; ++i) {
      d[i] = std::sqrt(v[i]);
      diag_h[i] = g[i] * dv[i];
      g_h[i] = d[i] * g[i];
    }
    for (int r = 0; r < M; ++r) f_aug[r] = f[r];
    MatMN<R, N> j_aug{};
    MatMN<M, N> j_h{};
    for (int r = 0; r < M; ++r)
      for (int c = 0; c < N; ++c) j_h[r][c] = j_aug[r][c] = jac[r][c] * d[c];
    for (int r = 0; r < N; ++r)
      for (int c = 0; c < N; ++c) j_aug[M + r][c] = r == c ? std::sqrt(diag_h[r]) : 0.0;
    MatMN<R, N> u{};
    VecN<N> s{};
    MatMN<N, N> vmat{};
    svd<R, N>(j_aug, u, s, vmat);
    VecN<N> uf{};
    for (int c = 0; c < N; ++c) {
      double acc = 0.0;
      for (int r = 0; r < R; ++r) acc += u[r][c] * f_aug[r];
      uf[c] = acc;
    }
    const double theta = std::max(0.995, 1.0 - g_norm);
    double actual_reduction = -1.0;
    VecN<N> x_new{};
    VecN<M> f_new{};
    double cost_new = 0.0;
    while (actual_reduction <= 0.0 && nfev < max_nfev) {
      const VecN<N> p_h = solve_lsq_trust_region<N>(M, uf, s, vmat, delta, alpha);
      VecN<N> p{};
      for (int i = 0; i < N; ++i) p[i] = d[i] * p_h[i];
      VecN<N> step{}, step_h{};
      const double predicted = select_step<M, N>(x, j_h, diag_h, g_h, p, p_h, d, delta, lb, ub, theta, step, step_h);
      VecN<N> trial{};
      for (int i = 0; i < N; ++i) trial[i] = x[i] + step[i];
      x_new = make_strictly_feasible<N>(trial, lb, ub, 0.0);
      fun(x_new, f_new);
      ++nfev;
      const double step_h_norm = norm<N>(step_h);
      bool finite = true;
      for (int r = 0; r < M; ++r) finite = finite && std::isfinite(f_new[r]);
      if (!finite) {
        delta = 0.25 * step_h_norm;
        continue;
      }
      cost_new = 0.5 * dot<M>(f_new, f_new);
      actual_reduction = cost - cost_new;
      // update_tr_radius
      double ratio;
      if (predicted > 0.0)
        ratio = actual_reduction / predicted;
      else if (predicted == actual_reduction && actual_reduction == 0.0)
        ratio = 1.0;
      else
        ratio = 0.0;
      double delta_new = delta;
      if (ratio < 0.25)
        delta_new = 0.25 * step_h_norm;
      else if (ratio > 0.75 && step_h_norm > 0.95 * delta)
        delta_new = delta * 2.0;
      // check_termination
      const double step_norm = norm<N>(step);
      const bool ftol_ok = actual_reduction < TOL * cost && ratio > 0.25;
      const bool xtol_ok = step_norm < TOL * (TOL + norm<N>(x));
      if (ftol_ok || xtol_ok) {
        done = true;
        break;
      }
      alpha *= delta / delta_new;
      delta = delta_new;
    }
    if (actual_reduction > 0.0) {
      x = x_new;
      f = f_new;
      cost = cost_new;
      jac = jacobian<M, N>(fun, x, f, lb, ub);
      g = grad();
    }
  }
  return x;
}

}  // namespace ik_reach::trf
