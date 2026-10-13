// Bounded nonlinear least squares: a step-for-step port of scipy.optimize.least_squares(method="trf") with its
// defaults as ikpy calls it (jac="2-point", x_scale=1, loss="linear", tr_solver="exact", ftol=xtol=gtol=1e-8,
// max_nfev=100*n). Following scipy's iterations (not just converging to the same minimum) is what keeps the planner's
// IK answers within float rounding of the Python ones: ikpy stops at scipy's tolerances, so a different solver would
// land elsewhere inside them.
#pragma once

#include <array>
#include <functional>

namespace grasp {

constexpr int TRF_MAX_N = 6;  // variables
constexpr int TRF_MAX_M = 6;  // residuals

using TrfVecN = std::array<double, TRF_MAX_N>;
using TrfVecM = std::array<double, TRF_MAX_M>;
// Residual function: fills f[0..m) for x[0..n).
using TrfResidual = std::function<void(const double *x, double *f)>;

struct TrfResult {
  TrfVecN x{};
  int nfev = 0;
  int status = 0;  // scipy termination status (0 max_nfev, 1 gtol, 2 ftol, 3 xtol, 4 ftol and xtol)
};

struct TrfOptions {
  double ftol = 1e-8;
  double xtol = 1e-8;
  double gtol = 1e-8;
};

// Minimize 0.5 * |fun(x)|^2 subject to lb <= x <= ub (finite bounds) from x0 (inside the bounds).
TrfResult least_squares_trf(const TrfResidual &fun, int n, int m, const double *x0, const double *lb, const double *ub,
                            const TrfOptions &options = TrfOptions());

}  // namespace grasp
