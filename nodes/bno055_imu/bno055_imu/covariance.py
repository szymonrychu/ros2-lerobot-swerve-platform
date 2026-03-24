"""Rolling-window 3D sample covariance estimator for IMU sensor axes."""

from collections import deque


class CovarianceEstimator:
    """Estimate 3x3 row-major covariance from a rolling window of 3D samples.

    Collects up to `window` most-recent (x, y, z) samples and computes the
    unbiased sample covariance matrix.  Returns None until `min_samples` have
    been added so the caller can fall back to a config-provided value.

    Attributes:
        window: Maximum number of samples retained.
        min_samples: Minimum samples required before covariance is returned.
    """

    def __init__(self, window: int, min_samples: int) -> None:
        """Initialise the estimator.

        Args:
            window: Rolling window size (positive integer).
            min_samples: Number of samples required before covariance() returns a value.
        """
        self.window = max(2, window)
        self.min_samples = max(2, min(min_samples, self.window))
        self._buf: deque[tuple[float, float, float]] = deque(maxlen=self.window)

    def add(self, x: float, y: float, z: float) -> None:
        """Append a 3D sample to the rolling buffer.

        Args:
            x: First axis value.
            y: Second axis value.
            z: Third axis value.
        """
        self._buf.append((float(x), float(y), float(z)))

    def covariance(self) -> list[float] | None:
        """Compute 9-element row-major 3x3 sample covariance from buffered samples.

        Returns:
            list[float] | None: 9-element row-major covariance matrix, or None if
                fewer than min_samples have been collected.
        """
        n = len(self._buf)
        if n < self.min_samples:
            return None

        mx = sum(s[0] for s in self._buf) / n
        my = sum(s[1] for s in self._buf) / n
        mz = sum(s[2] for s in self._buf) / n

        cxx = cxy = cxz = cyy = cyz = czz = 0.0
        for s in self._buf:
            dx = s[0] - mx
            dy = s[1] - my
            dz = s[2] - mz
            cxx += dx * dx
            cxy += dx * dy
            cxz += dx * dz
            cyy += dy * dy
            cyz += dy * dz
            czz += dz * dz

        denom = float(n - 1)
        cxx /= denom
        cxy /= denom
        cxz /= denom
        cyy /= denom
        cyz /= denom
        czz /= denom

        return [
            cxx,
            cxy,
            cxz,
            cxy,
            cyy,
            cyz,
            cxz,
            cyz,
            czz,
        ]
