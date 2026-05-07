"""
Compare KF, EKF, and UKF on the same 2D tracking problem.

State:
    x = [px, py, vx, vy]

Motion model:
    Constant velocity, linear.

Measurement model:
    Radar-like polar measurement, nonlinear:
    z = [range, bearing]
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def normalize_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """Map angles to [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def h_polar(state: np.ndarray) -> np.ndarray:
    """Nonlinear measurement function: [px, py, vx, vy] -> [range, bearing]."""
    px, py = state[0], state[1]
    distance = np.sqrt(px * px + py * py)
    bearing = np.arctan2(py, px)
    return np.array([distance, bearing])


def jacobian_h_polar(state: np.ndarray) -> np.ndarray:
    """Jacobian of the polar measurement function used by EKF."""
    px, py = state[0], state[1]
    squared_distance = max(px * px + py * py, 1e-9)
    distance = np.sqrt(squared_distance)

    return np.array(
        [
            [px / distance, py / distance, 0.0, 0.0],
            [-py / squared_distance, px / squared_distance, 0.0, 0.0],
        ]
    )


def polar_to_cartesian(measurement: np.ndarray) -> np.ndarray:
    """Convert [range, bearing] into a noisy Cartesian position [px, py]."""
    distance, bearing = measurement
    return np.array([distance * np.cos(bearing), distance * np.sin(bearing)])


@dataclass
class SimulationResult:
    true_states: np.ndarray
    polar_measurements: np.ndarray
    cartesian_measurements: np.ndarray
    dt: float


def simulate_data(steps: int = 120, dt: float = 1.0, seed: int = 7) -> SimulationResult:
    rng = np.random.default_rng(seed)

    true_states = np.zeros((steps, 4))
    polar_measurements = np.zeros((steps, 2))

    state = np.array([0.0, 0.0, 1.2, 0.35])
    process_std = np.array([0.04, 0.04, 0.03, 0.03])
    range_std = 1.8
    bearing_std = np.deg2rad(3.0)

    for k in range(steps):
        # A gently turning trajectory makes the nonlinear measurement meaningful.
        acceleration = np.array(
            [
                0.035 * np.sin(k / 14.0),
                0.028 * np.cos(k / 18.0),
            ]
        )
        state[2:] += acceleration * dt
        state[:2] += state[2:] * dt + rng.normal(0.0, process_std[:2])
        state[2:] += rng.normal(0.0, process_std[2:])

        true_states[k] = state
        clean_measurement = h_polar(state)
        polar_measurements[k] = clean_measurement + np.array(
            [
                rng.normal(0.0, range_std),
                rng.normal(0.0, bearing_std),
            ]
        )
        polar_measurements[k, 1] = normalize_angle(polar_measurements[k, 1])

    cartesian_measurements = np.array([polar_to_cartesian(z) for z in polar_measurements])
    return SimulationResult(true_states, polar_measurements, cartesian_measurements, dt)


class KalmanFilter2D:
    """Standard KF using Cartesian measurements converted from polar readings."""

    def __init__(self, dt: float):
        self.x = np.array([1.0, 1.0, 0.0, 0.0])
        self.P = np.diag([30.0, 30.0, 10.0, 10.0])
        self.F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ]
        )
        self.Q = np.diag([0.08, 0.08, 0.05, 0.05])
        self.R = np.diag([3.5, 3.5])

    def predict(self) -> None:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement_xy: np.ndarray) -> None:
        innovation = measurement_xy - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P


class ExtendedKalmanFilter2D:
    """EKF using a nonlinear polar measurement and its Jacobian."""

    def __init__(self, dt: float):
        self.x = np.array([1.0, 1.0, 0.0, 0.0])
        self.P = np.diag([30.0, 30.0, 10.0, 10.0])
        self.F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.Q = np.diag([0.08, 0.08, 0.05, 0.05])
        self.R = np.diag([1.8**2, np.deg2rad(3.0) ** 2])

    def predict(self) -> None:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement_polar: np.ndarray) -> None:
        H = jacobian_h_polar(self.x)
        innovation = measurement_polar - h_polar(self.x)
        innovation[1] = normalize_angle(innovation[1])
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ H) @ self.P


class UnscentedKalmanFilter2D:
    """UKF using sigma points to handle the nonlinear polar measurement."""

    def __init__(self, dt: float, alpha: float = 0.3, beta: float = 2.0, kappa: float = 0.0):
        self.n = 4
        self.m = 2
        self.x = np.array([1.0, 1.0, 0.0, 0.0])
        self.P = np.diag([30.0, 30.0, 10.0, 10.0])
        self.F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.Q = np.diag([0.08, 0.08, 0.05, 0.05])
        self.R = np.diag([1.8**2, np.deg2rad(3.0) ** 2])

        self.lambda_ = alpha * alpha * (self.n + kappa) - self.n
        scale = self.n + self.lambda_
        self.weights_mean = np.full(2 * self.n + 1, 1.0 / (2.0 * scale))
        self.weights_cov = np.full(2 * self.n + 1, 1.0 / (2.0 * scale))
        self.weights_mean[0] = self.lambda_ / scale
        self.weights_cov[0] = self.lambda_ / scale + (1.0 - alpha * alpha + beta)

    def sigma_points(self, mean: np.ndarray, covariance: np.ndarray) -> np.ndarray:
        covariance = 0.5 * (covariance + covariance.T)
        jitter = 1e-9 * np.eye(self.n)
        sqrt_matrix = np.linalg.cholesky((self.n + self.lambda_) * (covariance + jitter))
        points = [mean]
        for i in range(self.n):
            points.append(mean + sqrt_matrix[:, i])
            points.append(mean - sqrt_matrix[:, i])
        return np.array(points)

    def predict(self) -> None:
        points = self.sigma_points(self.x, self.P)
        predicted_points = np.array([self.F @ point for point in points])
        self.x = np.sum(self.weights_mean[:, None] * predicted_points, axis=0)

        self.P = self.Q.copy()
        for i, point in enumerate(predicted_points):
            diff = point - self.x
            self.P += self.weights_cov[i] * np.outer(diff, diff)

    def update(self, measurement_polar: np.ndarray) -> None:
        points = self.sigma_points(self.x, self.P)
        measurement_points = np.array([h_polar(point) for point in points])

        z_mean = np.sum(self.weights_mean[:, None] * measurement_points, axis=0)
        sin_mean = np.sum(self.weights_mean * np.sin(measurement_points[:, 1]))
        cos_mean = np.sum(self.weights_mean * np.cos(measurement_points[:, 1]))
        z_mean[1] = np.arctan2(sin_mean, cos_mean)

        Pzz = self.R.copy()
        Pxz = np.zeros((self.n, self.m))
        for i, point in enumerate(points):
            state_diff = point - self.x
            measurement_diff = measurement_points[i] - z_mean
            measurement_diff[1] = normalize_angle(measurement_diff[1])
            Pzz += self.weights_cov[i] * np.outer(measurement_diff, measurement_diff)
            Pxz += self.weights_cov[i] * np.outer(state_diff, measurement_diff)

        innovation = measurement_polar - z_mean
        innovation[1] = normalize_angle(innovation[1])
        K = Pxz @ np.linalg.inv(Pzz)
        self.x = self.x + K @ innovation
        self.P = self.P - K @ Pzz @ K.T


def run_filters(data: SimulationResult) -> dict[str, np.ndarray]:
    filters = {
        "KF": KalmanFilter2D(data.dt),
        "EKF": ExtendedKalmanFilter2D(data.dt),
        "UKF": UnscentedKalmanFilter2D(data.dt),
    }
    estimates = {name: [] for name in filters}

    for z_polar, z_xy in zip(data.polar_measurements, data.cartesian_measurements):
        filters["KF"].predict()
        filters["KF"].update(z_xy)
        estimates["KF"].append(filters["KF"].x.copy())

        filters["EKF"].predict()
        filters["EKF"].update(z_polar)
        estimates["EKF"].append(filters["EKF"].x.copy())

        filters["UKF"].predict()
        filters["UKF"].update(z_polar)
        estimates["UKF"].append(filters["UKF"].x.copy())

    return {name: np.array(values) for name, values in estimates.items()}


def position_error(true_states: np.ndarray, estimates: np.ndarray) -> np.ndarray:
    return np.linalg.norm(true_states[:, :2] - estimates[:, :2], axis=1)


def rmse(true_states: np.ndarray, estimates: np.ndarray) -> float:
    errors = true_states[:, :2] - estimates[:, :2]
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def plot_results(data: SimulationResult, estimates: dict[str, np.ndarray], output_path: Path) -> None:
    colors = {
        "KF": "#1f77b4",
        "EKF": "#ff7f0e",
        "UKF": "#2ca02c",
    }
    rmses = {name: rmse(data.true_states, estimate) for name, estimate in estimates.items()}

    fig = plt.figure(figsize=(14, 10), constrained_layout=True)
    grid = fig.add_gridspec(2, 2)

    ax_trajectory = fig.add_subplot(grid[0, 0])
    ax_error = fig.add_subplot(grid[0, 1])
    ax_rmse = fig.add_subplot(grid[1, 0])
    ax_final = fig.add_subplot(grid[1, 1])

    ax_trajectory.plot(
        data.true_states[:, 0],
        data.true_states[:, 1],
        color="black",
        linewidth=2.5,
        label="True trajectory",
    )
    ax_trajectory.scatter(
        data.cartesian_measurements[:, 0],
        data.cartesian_measurements[:, 1],
        s=12,
        color="#9aa0a6",
        alpha=0.45,
        label="Noisy polar measurements converted to x/y",
    )
    for name, estimate in estimates.items():
        ax_trajectory.plot(
            estimate[:, 0],
            estimate[:, 1],
            color=colors[name],
            linewidth=1.8,
            label=f"{name} estimate",
        )
    ax_trajectory.set_title("Trajectory: nonlinear polar measurements")
    ax_trajectory.set_xlabel("x position")
    ax_trajectory.set_ylabel("y position")
    ax_trajectory.axis("equal")
    ax_trajectory.grid(True, alpha=0.25)
    ax_trajectory.legend(loc="best", fontsize=8)

    steps = np.arange(len(data.true_states))
    for name, estimate in estimates.items():
        errors = position_error(data.true_states, estimate)
        ax_error.plot(steps, errors, color=colors[name], label=f"{name} error")
    ax_error.set_title("Position Error Over Time")
    ax_error.set_xlabel("time step")
    ax_error.set_ylabel("position error")
    ax_error.grid(True, alpha=0.25)
    ax_error.legend(loc="best", fontsize=8)

    names = list(estimates.keys())
    bars = ax_rmse.bar(names, [rmses[name] for name in names], color=[colors[name] for name in names])
    ax_rmse.set_title("Overall RMSE: lower is better")
    ax_rmse.set_ylabel("RMSE")
    ax_rmse.grid(True, axis="y", alpha=0.25)
    for bar, name in zip(bars, names):
        ax_rmse.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height(),
            f"{rmses[name]:.2f}",
            ha="center",
            va="bottom",
        )

    final_true = data.true_states[-1, :2]
    ax_final.scatter(final_true[0], final_true[1], color="black", s=90, marker="*", label="True final")
    for name, estimate in estimates.items():
        final_estimate = estimate[-1, :2]
        ax_final.scatter(
            final_estimate[0],
            final_estimate[1],
            color=colors[name],
            s=70,
            label=f"{name} final",
        )
        ax_final.plot(
            [final_true[0], final_estimate[0]],
            [final_true[1], final_estimate[1]],
            color=colors[name],
            linestyle="--",
            alpha=0.7,
        )
    ax_final.set_title("Final Estimate Offset")
    ax_final.set_xlabel("x position")
    ax_final.set_ylabel("y position")
    ax_final.axis("equal")
    ax_final.grid(True, alpha=0.25)
    ax_final.legend(loc="best", fontsize=8)

    fig.suptitle(
        "KF vs EKF vs UKF\n"
        "KF: linear model with converted x/y measurements | "
        "EKF: Jacobian linearization | "
        "UKF: sigma-point nonlinear update",
        fontsize=14,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> None:
    current_dir = Path(__file__).resolve().parent
    output_path = current_dir / "outputs" / "kalman_filters_comparison.png"

    data = simulate_data()
    estimates = run_filters(data)
    plot_results(data, estimates, output_path)

    print("Kalman filter comparison")
    print("========================")
    for name, estimate in estimates.items():
        print(f"{name:>3} RMSE: {rmse(data.true_states, estimate):.3f}")
    print()
    print(f"Figure saved to: {output_path}")


if __name__ == "__main__":
    main()
