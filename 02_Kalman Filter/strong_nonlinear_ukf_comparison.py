"""
Strong nonlinear comparison for KF, EKF, and UKF.

This example intentionally makes the measurement model harder:
    z = [range_from_origin, bearing_from_origin, range_from_landmark]

The extra landmark range creates stronger curvature in the measurement
function. EKF sees only a local tangent through the Jacobian, while UKF
propagates sigma points through the nonlinear measurement.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np


LANDMARK = np.array([55.0, -35.0])
DT = 1.0


def normalize_angle(angle: float | np.ndarray) -> float | np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def motion_model(state: np.ndarray, dt: float = DT) -> np.ndarray:
    """Nonlinear coordinated-turn motion model."""
    px, py, speed, heading, turn_rate = state
    turn_rate = float(turn_rate)

    if abs(turn_rate) < 1e-5:
        next_px = px + speed * np.cos(heading) * dt
        next_py = py + speed * np.sin(heading) * dt
    else:
        next_px = px + speed / turn_rate * (
            np.sin(heading + turn_rate * dt) - np.sin(heading)
        )
        next_py = py + speed / turn_rate * (
            -np.cos(heading + turn_rate * dt) + np.cos(heading)
        )

    next_heading = normalize_angle(heading + turn_rate * dt)
    return np.array([next_px, next_py, speed, next_heading, turn_rate])


def measurement_model(state: np.ndarray) -> np.ndarray:
    """Strong nonlinear measurement: origin range/bearing plus landmark range."""
    px, py = state[0], state[1]
    origin_range = np.hypot(px, py)
    origin_bearing = np.arctan2(py, px)
    landmark_range = np.hypot(px - LANDMARK[0], py - LANDMARK[1])
    return np.array([origin_range, origin_bearing, landmark_range])


def jacobian(function, state: np.ndarray, output_size: int, eps: float = 1e-5) -> np.ndarray:
    """Numerical Jacobian keeps the EKF readable for a learning script."""
    result = np.zeros((output_size, len(state)))
    for i in range(len(state)):
        step = np.zeros_like(state)
        step[i] = eps
        plus = function(state + step)
        minus = function(state - step)
        diff = plus - minus
        if output_size >= 2:
            diff[1] = normalize_angle(diff[1])
        result[:, i] = diff / (2.0 * eps)
    return result


def polar_to_cartesian(measurement: np.ndarray) -> np.ndarray:
    distance, bearing = measurement[0], measurement[1]
    return np.array([distance * np.cos(bearing), distance * np.sin(bearing)])


@dataclass
class StrongSimulation:
    true_states: np.ndarray
    measurements: np.ndarray
    cartesian_measurements: np.ndarray


def simulate_strong_nonlinear_data(steps: int = 180, seed: int = 11) -> StrongSimulation:
    rng = np.random.default_rng(seed)
    true_states = np.zeros((steps, 5))
    measurements = np.zeros((steps, 3))

    state = np.array([3.0, -45.0, 2.8, 1.25, 0.045])
    measurement_std = np.array([2.0, np.deg2rad(2.5), 1.2])

    for k in range(steps):
        # Force changing speed and turn rate to make the trajectory and
        # measurement geometry strongly nonlinear.
        state[2] += 0.06 * np.sin(k / 11.0) + rng.normal(0.0, 0.025)
        state[4] = 0.055 * np.sin(k / 18.0) + 0.025 * np.cos(k / 7.0)
        state = motion_model(state)
        state[0:2] += rng.normal(0.0, 0.05, size=2)
        state[3] = normalize_angle(state[3] + rng.normal(0.0, np.deg2rad(0.3)))

        true_states[k] = state
        measurements[k] = measurement_model(state) + rng.normal(0.0, measurement_std)
        measurements[k, 1] = normalize_angle(measurements[k, 1])

    cartesian_measurements = np.array([polar_to_cartesian(z) for z in measurements])
    return StrongSimulation(true_states, measurements, cartesian_measurements)


class LinearKalmanFilter:
    """Ordinary KF with a constant-velocity state and converted x/y measurements."""

    def __init__(self):
        self.x = np.array([0.0, -40.0, 0.0, 0.0])
        self.P = np.diag([60.0, 60.0, 20.0, 20.0])
        self.F = np.array(
            [
                [1.0, 0.0, DT, 0.0],
                [0.0, 1.0, 0.0, DT],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.Q = np.diag([0.25, 0.25, 0.18, 0.18])
        self.R = np.diag([7.0, 7.0])

    def step(self, measurement: np.ndarray) -> np.ndarray:
        z_xy = polar_to_cartesian(measurement)
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        innovation = z_xy - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return np.array([self.x[0], self.x[1]])


class ExtendedKalmanFilter:
    """EKF using local linearization of both motion and measurement models."""

    def __init__(self):
        self.x = np.array([0.0, -40.0, 2.0, 1.0, 0.02])
        self.P = np.diag([60.0, 60.0, 10.0, 1.0, 0.2])
        self.Q = np.diag([0.12, 0.12, 0.16, np.deg2rad(1.2) ** 2, 0.006])
        self.R = np.diag([2.0**2, np.deg2rad(2.5) ** 2, 1.2**2])

    def step(self, measurement: np.ndarray) -> np.ndarray:
        F = jacobian(motion_model, self.x, output_size=5)
        self.x = motion_model(self.x)
        self.P = F @ self.P @ F.T + self.Q

        H = jacobian(measurement_model, self.x, output_size=3)
        innovation = measurement - measurement_model(self.x)
        innovation[1] = normalize_angle(innovation[1])
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[3] = normalize_angle(self.x[3])
        self.P = (np.eye(5) - K @ H) @ self.P
        return self.x[:2].copy()


class UnscentedKalmanFilter:
    """UKF propagating sigma points through the nonlinear models."""

    def __init__(self, alpha: float = 0.45, beta: float = 2.0, kappa: float = 0.0):
        self.n = 5
        self.m = 3
        self.x = np.array([0.0, -40.0, 2.0, 1.0, 0.02])
        self.P = np.diag([60.0, 60.0, 10.0, 1.0, 0.2])
        self.Q = np.diag([0.12, 0.12, 0.16, np.deg2rad(1.2) ** 2, 0.006])
        self.R = np.diag([2.0**2, np.deg2rad(2.5) ** 2, 1.2**2])

        self.lambda_ = alpha * alpha * (self.n + kappa) - self.n
        scale = self.n + self.lambda_
        self.wm = np.full(2 * self.n + 1, 1.0 / (2.0 * scale))
        self.wc = np.full(2 * self.n + 1, 1.0 / (2.0 * scale))
        self.wm[0] = self.lambda_ / scale
        self.wc[0] = self.lambda_ / scale + (1.0 - alpha * alpha + beta)

    def sigma_points(self) -> np.ndarray:
        covariance = 0.5 * (self.P + self.P.T)
        jitter = 1e-8 * np.eye(self.n)
        sqrt_matrix = np.linalg.cholesky((self.n + self.lambda_) * (covariance + jitter))
        points = [self.x]
        for i in range(self.n):
            points.append(self.x + sqrt_matrix[:, i])
            points.append(self.x - sqrt_matrix[:, i])
        return np.array(points)

    def weighted_state_mean(self, points: np.ndarray) -> np.ndarray:
        mean = np.sum(self.wm[:, None] * points, axis=0)
        sin_heading = np.sum(self.wm * np.sin(points[:, 3]))
        cos_heading = np.sum(self.wm * np.cos(points[:, 3]))
        mean[3] = np.arctan2(sin_heading, cos_heading)
        return mean

    def step(self, measurement: np.ndarray) -> np.ndarray:
        sigma = self.sigma_points()
        predicted = np.array([motion_model(point) for point in sigma])
        self.x = self.weighted_state_mean(predicted)

        self.P = self.Q.copy()
        for i, point in enumerate(predicted):
            diff = point - self.x
            diff[3] = normalize_angle(diff[3])
            self.P += self.wc[i] * np.outer(diff, diff)

        sigma = self.sigma_points()
        predicted_measurements = np.array([measurement_model(point) for point in sigma])
        z_mean = np.sum(self.wm[:, None] * predicted_measurements, axis=0)
        sin_bearing = np.sum(self.wm * np.sin(predicted_measurements[:, 1]))
        cos_bearing = np.sum(self.wm * np.cos(predicted_measurements[:, 1]))
        z_mean[1] = np.arctan2(sin_bearing, cos_bearing)

        Pzz = self.R.copy()
        Pxz = np.zeros((self.n, self.m))
        for i, point in enumerate(sigma):
            state_diff = point - self.x
            state_diff[3] = normalize_angle(state_diff[3])
            measurement_diff = predicted_measurements[i] - z_mean
            measurement_diff[1] = normalize_angle(measurement_diff[1])
            Pzz += self.wc[i] * np.outer(measurement_diff, measurement_diff)
            Pxz += self.wc[i] * np.outer(state_diff, measurement_diff)

        innovation = measurement - z_mean
        innovation[1] = normalize_angle(innovation[1])
        K = Pxz @ np.linalg.inv(Pzz)
        self.x = self.x + K @ innovation
        self.x[3] = normalize_angle(self.x[3])
        self.P = self.P - K @ Pzz @ K.T
        return self.x[:2].copy()


def run_one_filter(filter_class, measurements: np.ndarray) -> tuple[np.ndarray, float]:
    filter_instance = filter_class()
    estimates = []
    start = perf_counter()
    for measurement in measurements:
        estimates.append(filter_instance.step(measurement))
    elapsed_ms = (perf_counter() - start) * 1000.0
    return np.array(estimates), elapsed_ms


def rmse(true_states: np.ndarray, estimates_xy: np.ndarray) -> float:
    errors = true_states[:, :2] - estimates_xy
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def mean_runtime_ms(filter_class, measurements: np.ndarray, repeats: int = 80) -> float:
    start = perf_counter()
    for _ in range(repeats):
        run_one_filter(filter_class, measurements)
    return (perf_counter() - start) * 1000.0 / repeats


def plot_results(
    data: StrongSimulation,
    estimates: dict[str, np.ndarray],
    rmses: dict[str, float],
    runtimes: dict[str, float],
    output_path: Path,
) -> None:
    colors = {"KF": "#1f77b4", "EKF": "#ff7f0e", "UKF": "#2ca02c"}
    fig = plt.figure(figsize=(15, 10), constrained_layout=True)
    grid = fig.add_gridspec(2, 2)
    ax_track = fig.add_subplot(grid[0, 0])
    ax_error = fig.add_subplot(grid[0, 1])
    ax_rmse = fig.add_subplot(grid[1, 0])
    ax_time = fig.add_subplot(grid[1, 1])

    ax_track.plot(data.true_states[:, 0], data.true_states[:, 1], color="black", linewidth=2.4, label="True")
    ax_track.scatter(
        data.cartesian_measurements[:, 0],
        data.cartesian_measurements[:, 1],
        color="#9aa0a6",
        s=10,
        alpha=0.35,
        label="Noisy origin polar -> x/y",
    )
    ax_track.scatter(LANDMARK[0], LANDMARK[1], color="#d62728", marker="X", s=90, label="Landmark")
    for name, estimate in estimates.items():
        ax_track.plot(estimate[:, 0], estimate[:, 1], color=colors[name], linewidth=1.8, label=name)
    ax_track.set_title("Strong Nonlinear Tracking")
    ax_track.set_xlabel("x position")
    ax_track.set_ylabel("y position")
    ax_track.axis("equal")
    ax_track.grid(True, alpha=0.25)
    ax_track.legend(fontsize=8)

    steps = np.arange(len(data.true_states))
    for name, estimate in estimates.items():
        error = np.linalg.norm(data.true_states[:, :2] - estimate, axis=1)
        ax_error.plot(steps, error, color=colors[name], label=f"{name} error")
    ax_error.set_title("Position Error Over Time")
    ax_error.set_xlabel("time step")
    ax_error.set_ylabel("position error")
    ax_error.grid(True, alpha=0.25)
    ax_error.legend(fontsize=8)

    names = ["KF", "EKF", "UKF"]
    rmse_bars = ax_rmse.bar(names, [rmses[name] for name in names], color=[colors[name] for name in names])
    ax_rmse.set_title("Effect: RMSE lower is better")
    ax_rmse.set_ylabel("RMSE")
    ax_rmse.grid(True, axis="y", alpha=0.25)
    for bar, name in zip(rmse_bars, names):
        ax_rmse.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f"{rmses[name]:.2f}", ha="center", va="bottom")

    time_bars = ax_time.bar(names, [runtimes[name] for name in names], color=[colors[name] for name in names])
    ax_time.set_title("Cost: average runtime lower is faster")
    ax_time.set_ylabel("milliseconds per full run")
    ax_time.grid(True, axis="y", alpha=0.25)
    for bar, name in zip(time_bars, names):
        ax_time.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"{runtimes[name]:.2f} ms",
            ha="center",
            va="bottom",
        )

    fig.suptitle(
        "Strong Nonlinear Case: KF vs EKF vs UKF\n"
        "Measurement = origin range/bearing + landmark range; motion = coordinated turn",
        fontsize=14,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> None:
    current_dir = Path(__file__).resolve().parent
    output_path = current_dir / "outputs" / "strong_nonlinear_ukf_comparison.png"

    data = simulate_strong_nonlinear_data()
    filter_classes = {
        "KF": LinearKalmanFilter,
        "EKF": ExtendedKalmanFilter,
        "UKF": UnscentedKalmanFilter,
    }

    estimates = {}
    one_run_times = {}
    average_times = {}
    for name, filter_class in filter_classes.items():
        estimates[name], one_run_times[name] = run_one_filter(filter_class, data.measurements)
        average_times[name] = mean_runtime_ms(filter_class, data.measurements)

    rmses = {name: rmse(data.true_states, estimate) for name, estimate in estimates.items()}
    plot_results(data, estimates, rmses, average_times, output_path)

    print("Strong nonlinear Kalman comparison")
    print("==================================")
    for name in ["KF", "EKF", "UKF"]:
        print(
            f"{name:>3} RMSE: {rmses[name]:7.3f} | "
            f"one run: {one_run_times[name]:7.3f} ms | "
            f"avg run: {average_times[name]:7.3f} ms"
        )
    print()
    print(f"Figure saved to: {output_path}")


if __name__ == "__main__":
    main()
