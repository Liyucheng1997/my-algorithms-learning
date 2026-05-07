"""
Compare RRT, RRT-Connect, and RRT* in the same continuous 2D world.

The map uses rectangular obstacles. Each planner returns the final path,
the sampled tree edges, node count, and runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


Point = np.ndarray
Bounds = tuple[float, float, float, float]
Obstacle = tuple[float, float, float, float]


@dataclass
class Node:
    point: Point
    parent: int | None
    cost: float = 0.0


@dataclass
class PlannerResult:
    name: str
    path: list[Point]
    edges: list[tuple[Point, Point]]
    node_count: int
    elapsed_ms: float

    @property
    def found(self) -> bool:
        return len(self.path) > 0

    @property
    def path_length(self) -> float:
        if len(self.path) < 2:
            return 0.0
        return float(sum(distance(a, b) for a, b in zip(self.path[:-1], self.path[1:])))


def distance(a: Point, b: Point) -> float:
    return float(np.linalg.norm(a - b))


def make_world() -> tuple[Bounds, list[Obstacle], Point, Point]:
    bounds = (0.0, 100.0, 0.0, 70.0)
    obstacles = [
        (18.0, 8.0, 10.0, 42.0),
        (38.0, 20.0, 9.0, 42.0),
        (58.0, 0.0, 10.0, 38.0),
        (73.0, 28.0, 9.0, 34.0),
        (8.0, 55.0, 34.0, 7.0),
        (50.0, 48.0, 27.0, 7.0),
    ]
    start = np.array([7.0, 8.0])
    goal = np.array([93.0, 62.0])
    return bounds, obstacles, start, goal


def point_in_obstacle(point: Point, obstacle: Obstacle) -> bool:
    x, y = point
    ox, oy, width, height = obstacle
    return ox <= x <= ox + width and oy <= y <= oy + height


def point_is_free(point: Point, bounds: Bounds, obstacles: list[Obstacle]) -> bool:
    min_x, max_x, min_y, max_y = bounds
    if not (min_x <= point[0] <= max_x and min_y <= point[1] <= max_y):
        return False
    return not any(point_in_obstacle(point, obstacle) for obstacle in obstacles)


def edge_is_free(a: Point, b: Point, bounds: Bounds, obstacles: list[Obstacle], resolution: float = 1.0) -> bool:
    steps = max(int(distance(a, b) / resolution), 1)
    for alpha in np.linspace(0.0, 1.0, steps + 1):
        point = a + alpha * (b - a)
        if not point_is_free(point, bounds, obstacles):
            return False
    return True


def steer(from_point: Point, to_point: Point, step_size: float) -> Point:
    direction = to_point - from_point
    length = np.linalg.norm(direction)
    if length <= step_size:
        return to_point.copy()
    return from_point + direction / length * step_size


def sample_point(rng: np.random.Generator, bounds: Bounds, goal: Point, goal_sample_rate: float) -> Point:
    if rng.random() < goal_sample_rate:
        return goal.copy()
    min_x, max_x, min_y, max_y = bounds
    return np.array([rng.uniform(min_x, max_x), rng.uniform(min_y, max_y)])


def nearest_node_index(nodes: list[Node], point: Point) -> int:
    distances = [distance(node.point, point) for node in nodes]
    return int(np.argmin(distances))


def near_node_indices(nodes: list[Node], point: Point, radius: float) -> list[int]:
    return [index for index, node in enumerate(nodes) if distance(node.point, point) <= radius]


def extract_path(nodes: list[Node], index: int) -> list[Point]:
    path = []
    current = index
    while current is not None:
        node = nodes[current]
        path.append(node.point)
        current = node.parent
    path.reverse()
    return path


def rrt(
    bounds: Bounds,
    obstacles: list[Obstacle],
    start: Point,
    goal: Point,
    seed: int = 4,
    max_iter: int = 2500,
    step_size: float = 4.0,
    goal_radius: float = 5.0,
    goal_sample_rate: float = 0.08,
) -> PlannerResult:
    rng = np.random.default_rng(seed)
    nodes = [Node(start.copy(), None, 0.0)]
    edges: list[tuple[Point, Point]] = []

    begin = perf_counter()
    goal_index: int | None = None
    for _ in range(max_iter):
        sample = sample_point(rng, bounds, goal, goal_sample_rate)
        nearest_index = nearest_node_index(nodes, sample)
        nearest = nodes[nearest_index]
        new_point = steer(nearest.point, sample, step_size)

        if not edge_is_free(nearest.point, new_point, bounds, obstacles):
            continue

        nodes.append(Node(new_point, nearest_index, nearest.cost + distance(nearest.point, new_point)))
        new_index = len(nodes) - 1
        edges.append((nearest.point.copy(), new_point.copy()))

        if distance(new_point, goal) <= goal_radius and edge_is_free(new_point, goal, bounds, obstacles):
            nodes.append(Node(goal.copy(), new_index, nodes[new_index].cost + distance(new_point, goal)))
            goal_index = len(nodes) - 1
            edges.append((new_point.copy(), goal.copy()))
            break

    elapsed_ms = (perf_counter() - begin) * 1000.0
    path = extract_path(nodes, goal_index) if goal_index is not None else []
    return PlannerResult("RRT", path, edges, len(nodes), elapsed_ms)


def build_path_between_trees(
    tree_a: list[Node],
    index_a: int,
    tree_b: list[Node],
    index_b: int,
    start_tree_is_a: bool,
) -> list[Point]:
    path_a = extract_path(tree_a, index_a)
    path_b = extract_path(tree_b, index_b)
    if start_tree_is_a:
        return path_a + list(reversed(path_b))
    return path_b + list(reversed(path_a))


def extend_tree(
    tree: list[Node],
    target: Point,
    bounds: Bounds,
    obstacles: list[Obstacle],
    step_size: float,
    edges: list[tuple[Point, Point]],
) -> tuple[int | None, bool]:
    nearest_index = nearest_node_index(tree, target)
    nearest = tree[nearest_index]
    new_point = steer(nearest.point, target, step_size)

    if not edge_is_free(nearest.point, new_point, bounds, obstacles):
        return None, False

    tree.append(Node(new_point, nearest_index, nearest.cost + distance(nearest.point, new_point)))
    new_index = len(tree) - 1
    edges.append((nearest.point.copy(), new_point.copy()))
    reached = distance(new_point, target) < 1e-9
    return new_index, reached


def rrt_connect(
    bounds: Bounds,
    obstacles: list[Obstacle],
    start: Point,
    goal: Point,
    seed: int = 6,
    max_iter: int = 1400,
    step_size: float = 4.5,
) -> PlannerResult:
    rng = np.random.default_rng(seed)
    tree_start = [Node(start.copy(), None, 0.0)]
    tree_goal = [Node(goal.copy(), None, 0.0)]
    edges: list[tuple[Point, Point]] = []

    begin = perf_counter()
    path: list[Point] = []
    start_tree_active = True

    for _ in range(max_iter):
        random_point = sample_point(rng, bounds, goal, goal_sample_rate=0.03)
        active = tree_start if start_tree_active else tree_goal
        other = tree_goal if start_tree_active else tree_start

        active_index, _ = extend_tree(active, random_point, bounds, obstacles, step_size, edges)
        if active_index is None:
            start_tree_active = not start_tree_active
            continue

        target = active[active_index].point
        last_other_index: int | None = None
        reached = False
        while True:
            other_index, reached_target = extend_tree(other, target, bounds, obstacles, step_size, edges)
            if other_index is None:
                break
            last_other_index = other_index
            if reached_target or distance(other[other_index].point, target) <= 1e-9:
                reached = True
                break

        if reached and last_other_index is not None:
            path = build_path_between_trees(
                active,
                active_index,
                other,
                last_other_index,
                start_tree_is_a=start_tree_active,
            )
            break

        start_tree_active = not start_tree_active

    elapsed_ms = (perf_counter() - begin) * 1000.0
    return PlannerResult("RRT-Connect", path, edges, len(tree_start) + len(tree_goal), elapsed_ms)


def rrt_star(
    bounds: Bounds,
    obstacles: list[Obstacle],
    start: Point,
    goal: Point,
    seed: int = 8,
    max_iter: int = 1900,
    step_size: float = 4.0,
    goal_radius: float = 5.0,
    goal_sample_rate: float = 0.08,
    rewire_radius: float = 10.0,
) -> PlannerResult:
    rng = np.random.default_rng(seed)
    nodes = [Node(start.copy(), None, 0.0)]
    edges: list[tuple[Point, Point]] = []
    best_goal_index: int | None = None

    begin = perf_counter()
    for _ in range(max_iter):
        sample = sample_point(rng, bounds, goal, goal_sample_rate)
        nearest_index = nearest_node_index(nodes, sample)
        nearest = nodes[nearest_index]
        new_point = steer(nearest.point, sample, step_size)

        if not edge_is_free(nearest.point, new_point, bounds, obstacles):
            continue

        nearby = near_node_indices(nodes, new_point, rewire_radius)
        best_parent = nearest_index
        best_cost = nearest.cost + distance(nearest.point, new_point)
        for index in nearby:
            candidate = nodes[index]
            candidate_cost = candidate.cost + distance(candidate.point, new_point)
            if candidate_cost < best_cost and edge_is_free(candidate.point, new_point, bounds, obstacles):
                best_parent = index
                best_cost = candidate_cost

        nodes.append(Node(new_point, best_parent, best_cost))
        new_index = len(nodes) - 1
        edges.append((nodes[best_parent].point.copy(), new_point.copy()))

        # Rewire nearby nodes through the new node when it improves their cost.
        for index in nearby:
            candidate = nodes[index]
            new_cost = best_cost + distance(new_point, candidate.point)
            if new_cost < candidate.cost and edge_is_free(new_point, candidate.point, bounds, obstacles):
                candidate.parent = new_index
                candidate.cost = new_cost
                edges.append((new_point.copy(), candidate.point.copy()))

        if distance(new_point, goal) <= goal_radius and edge_is_free(new_point, goal, bounds, obstacles):
            goal_cost = best_cost + distance(new_point, goal)
            if best_goal_index is None:
                nodes.append(Node(goal.copy(), new_index, goal_cost))
                best_goal_index = len(nodes) - 1
                edges.append((new_point.copy(), goal.copy()))
            elif goal_cost < nodes[best_goal_index].cost:
                nodes[best_goal_index].parent = new_index
                nodes[best_goal_index].cost = goal_cost
                edges.append((new_point.copy(), goal.copy()))

    elapsed_ms = (perf_counter() - begin) * 1000.0
    path = extract_path(nodes, best_goal_index) if best_goal_index is not None else []
    return PlannerResult("RRT*", path, edges, len(nodes), elapsed_ms)


def run_all_planners(bounds: Bounds, obstacles: list[Obstacle], start: Point, goal: Point) -> list[PlannerResult]:
    return [
        rrt(bounds, obstacles, start, goal),
        rrt_connect(bounds, obstacles, start, goal),
        rrt_star(bounds, obstacles, start, goal),
    ]


def draw_world(
    ax: plt.Axes,
    bounds: Bounds,
    obstacles: list[Obstacle],
    start: Point,
    goal: Point,
    result: PlannerResult,
) -> None:
    min_x, max_x, min_y, max_y = bounds
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="box")

    for ox, oy, width, height in obstacles:
        ax.add_patch(Rectangle((ox, oy), width, height, color="#2b2b2b"))

    for a, b in result.edges:
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#9ecae1", linewidth=0.45, alpha=0.55)

    if result.path:
        path_array = np.array(result.path)
        ax.plot(path_array[:, 0], path_array[:, 1], color="#08519c", linewidth=3.0, label="Final path")

    ax.scatter(start[0], start[1], color="#16a34a", s=80, zorder=5, label="Start")
    ax.scatter(goal[0], goal[1], color="#dc2626", s=80, zorder=5, label="Goal")
    ax.set_title(
        f"{result.name}: length={result.path_length:.1f}, "
        f"nodes={result.node_count}, time={result.elapsed_ms:.1f} ms"
    )
    ax.grid(True, alpha=0.2)


def plot_results(
    bounds: Bounds,
    obstacles: list[Obstacle],
    start: Point,
    goal: Point,
    results: list[PlannerResult],
    output_path: Path,
) -> None:
    fig = plt.figure(figsize=(15, 10), constrained_layout=True)
    layout = fig.add_gridspec(2, 3)

    for index, result in enumerate(results):
        ax = fig.add_subplot(layout[0, index])
        draw_world(ax, bounds, obstacles, start, goal, result)

    names = [result.name for result in results]
    lengths = [result.path_length for result in results]
    node_counts = [result.node_count for result in results]
    runtimes = [result.elapsed_ms for result in results]
    colors = ["#4c78a8", "#72b7b2", "#54a24b"]

    ax_length = fig.add_subplot(layout[1, 0])
    ax_nodes = fig.add_subplot(layout[1, 1])
    ax_time = fig.add_subplot(layout[1, 2])

    ax_length.bar(names, lengths, color=colors)
    ax_length.set_title("Path Length")
    ax_length.set_ylabel("continuous distance")
    ax_length.grid(True, axis="y", alpha=0.25)

    ax_nodes.bar(names, node_counts, color=colors)
    ax_nodes.set_title("Tree Size")
    ax_nodes.set_ylabel("nodes")
    ax_nodes.grid(True, axis="y", alpha=0.25)

    ax_time.bar(names, runtimes, color=colors)
    ax_time.set_title("Runtime")
    ax_time.set_ylabel("milliseconds")
    ax_time.grid(True, axis="y", alpha=0.25)

    fig.suptitle(
        "RRT Family: RRT vs RRT-Connect vs RRT*\n"
        "Light blue = sampled tree edges, dark blue = final path",
        fontsize=14,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_results(results: list[PlannerResult]) -> None:
    print("RRT family comparison")
    print("=====================")
    print(f"{'Planner':<12} {'Found':<7} {'Length':>10} {'Nodes':>8} {'Time(ms)':>10}")
    for result in results:
        print(
            f"{result.name:<12} {str(result.found):<7} "
            f"{result.path_length:>10.2f} {result.node_count:>8} {result.elapsed_ms:>10.2f}"
        )


def main() -> None:
    current_dir = Path(__file__).resolve().parent
    output_path = current_dir / "outputs" / "rrt_family_comparison.png"

    bounds, obstacles, start, goal = make_world()
    results = run_all_planners(bounds, obstacles, start, goal)
    print_results(results)
    plot_results(bounds, obstacles, start, goal, results, output_path)
    print()
    print(f"Figure saved to: {output_path}")


if __name__ == "__main__":
    main()
