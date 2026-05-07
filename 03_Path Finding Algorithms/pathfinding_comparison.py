"""
Compare DFS, BFS, Dijkstra, and A* on the same grid map.

The grid is unweighted:
    0 = free cell
    1 = obstacle

Movement is 4-neighbor only: up, down, left, right.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from heapq import heappop, heappush
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np


Position = tuple[int, int]


@dataclass
class SearchResult:
    name: str
    path: list[Position]
    visited_order: list[Position]
    elapsed_ms: float

    @property
    def found(self) -> bool:
        return len(self.path) > 0

    @property
    def path_steps(self) -> int:
        return max(len(self.path) - 1, 0)

    @property
    def visited_count(self) -> int:
        return len(self.visited_order)


def make_demo_grid() -> tuple[np.ndarray, Position, Position]:
    """Create a deterministic maze-like grid with a guaranteed path."""
    height, width = 25, 35
    grid = np.zeros((height, width), dtype=int)

    # Border walls.
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    # Internal walls with openings. This creates a map where the search
    # strategy visibly changes how many nodes get expanded.
    grid[3:22, 6] = 1
    grid[3, 6] = 0
    grid[16, 6] = 0

    grid[2:19, 12] = 1
    grid[8, 12] = 0
    grid[18, 12] = 0

    grid[6:24, 18] = 1
    grid[7, 18] = 0
    grid[20, 18] = 0

    grid[1:18, 25] = 1
    grid[5, 25] = 0
    grid[17, 25] = 0

    grid[6, 2:12] = 1
    grid[6, 4] = 0
    grid[6, 10] = 0

    grid[12, 7:25] = 1
    grid[12, 9] = 0
    grid[12, 16] = 0
    grid[12, 23] = 0

    grid[19, 13:33] = 1
    grid[19, 15] = 0
    grid[19, 27] = 0

    grid[9:16, 29] = 1
    grid[10, 29] = 0
    grid[15, 29] = 0

    start = (2, 2)
    goal = (22, 32)
    grid[start] = 0
    grid[goal] = 0
    return grid, start, goal


def neighbors(grid: np.ndarray, position: Position) -> list[Position]:
    row, col = position
    candidates = [
        (row - 1, col),
        (row, col + 1),
        (row + 1, col),
        (row, col - 1),
    ]
    result = []
    for next_row, next_col in candidates:
        if grid[next_row, next_col] == 0:
            result.append((next_row, next_col))
    return result


def reconstruct_path(parent: dict[Position, Position], start: Position, goal: Position) -> list[Position]:
    if goal not in parent and goal != start:
        return []

    path = [goal]
    current = goal
    while current != start:
        current = parent[current]
        path.append(current)
    path.reverse()
    return path


def depth_first_search(grid: np.ndarray, start: Position, goal: Position) -> SearchResult:
    begin = perf_counter()
    stack = [start]
    parent: dict[Position, Position] = {}
    seen = {start}
    visited_order: list[Position] = []

    while stack:
        current = stack.pop()
        visited_order.append(current)
        if current == goal:
            break

        for next_position in neighbors(grid, current):
            if next_position not in seen:
                seen.add(next_position)
                parent[next_position] = current
                stack.append(next_position)

    elapsed_ms = (perf_counter() - begin) * 1000.0
    return SearchResult("DFS", reconstruct_path(parent, start, goal), visited_order, elapsed_ms)


def breadth_first_search(grid: np.ndarray, start: Position, goal: Position) -> SearchResult:
    begin = perf_counter()
    queue = deque([start])
    parent: dict[Position, Position] = {}
    seen = {start}
    visited_order: list[Position] = []

    while queue:
        current = queue.popleft()
        visited_order.append(current)
        if current == goal:
            break

        for next_position in neighbors(grid, current):
            if next_position not in seen:
                seen.add(next_position)
                parent[next_position] = current
                queue.append(next_position)

    elapsed_ms = (perf_counter() - begin) * 1000.0
    return SearchResult("BFS", reconstruct_path(parent, start, goal), visited_order, elapsed_ms)


def dijkstra_search(grid: np.ndarray, start: Position, goal: Position) -> SearchResult:
    begin = perf_counter()
    heap: list[tuple[int, Position]] = [(0, start)]
    parent: dict[Position, Position] = {}
    best_cost = {start: 0}
    visited_order: list[Position] = []
    expanded: set[Position] = set()

    while heap:
        current_cost, current = heappop(heap)
        if current in expanded:
            continue
        expanded.add(current)
        visited_order.append(current)

        if current == goal:
            break

        for next_position in neighbors(grid, current):
            new_cost = current_cost + 1
            if new_cost < best_cost.get(next_position, float("inf")):
                best_cost[next_position] = new_cost
                parent[next_position] = current
                heappush(heap, (new_cost, next_position))

    elapsed_ms = (perf_counter() - begin) * 1000.0
    return SearchResult("Dijkstra", reconstruct_path(parent, start, goal), visited_order, elapsed_ms)


def manhattan_distance(a: Position, b: Position) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(grid: np.ndarray, start: Position, goal: Position) -> SearchResult:
    begin = perf_counter()
    heap: list[tuple[int, int, Position]] = [(manhattan_distance(start, goal), 0, start)]
    parent: dict[Position, Position] = {}
    best_cost = {start: 0}
    visited_order: list[Position] = []
    expanded: set[Position] = set()

    while heap:
        _, current_cost, current = heappop(heap)
        if current in expanded:
            continue
        expanded.add(current)
        visited_order.append(current)

        if current == goal:
            break

        for next_position in neighbors(grid, current):
            new_cost = current_cost + 1
            if new_cost < best_cost.get(next_position, float("inf")):
                best_cost[next_position] = new_cost
                parent[next_position] = current
                priority = new_cost + manhattan_distance(next_position, goal)
                heappush(heap, (priority, new_cost, next_position))

    elapsed_ms = (perf_counter() - begin) * 1000.0
    return SearchResult("A*", reconstruct_path(parent, start, goal), visited_order, elapsed_ms)


def run_all_algorithms(grid: np.ndarray, start: Position, goal: Position) -> list[SearchResult]:
    algorithms = [
        depth_first_search,
        breadth_first_search,
        dijkstra_search,
        a_star_search,
    ]
    return [algorithm(grid, start, goal) for algorithm in algorithms]


def draw_grid_result(
    ax: plt.Axes,
    grid: np.ndarray,
    start: Position,
    goal: Position,
    result: SearchResult,
) -> None:
    image = np.ones((*grid.shape, 3), dtype=float)
    image[grid == 1] = np.array([0.12, 0.12, 0.12])

    for row, col in result.visited_order:
        if grid[row, col] == 0:
            image[row, col] = np.array([0.78, 0.89, 1.0])

    for row, col in result.path:
        image[row, col] = np.array([0.12, 0.34, 0.82])

    image[start] = np.array([0.05, 0.62, 0.28])
    image[goal] = np.array([0.86, 0.12, 0.12])

    ax.imshow(image, interpolation="nearest")
    ax.set_title(
        f"{result.name}: steps={result.path_steps}, "
        f"visited={result.visited_count}, time={result.elapsed_ms:.3f} ms"
    )
    ax.set_xticks([])
    ax.set_yticks([])


def plot_results(
    grid: np.ndarray,
    start: Position,
    goal: Position,
    results: list[SearchResult],
    output_path: Path,
) -> None:
    fig = plt.figure(figsize=(14, 12), constrained_layout=True)
    layout = fig.add_gridspec(3, 4)

    panel_positions = [
        layout[0, 0:2],
        layout[0, 2:4],
        layout[1, 0:2],
        layout[1, 2:4],
    ]
    for index, result in enumerate(results):
        ax = fig.add_subplot(panel_positions[index])
        draw_grid_result(ax, grid, start, goal, result)

    names = [result.name for result in results]
    path_steps = [result.path_steps for result in results]
    visited_counts = [result.visited_count for result in results]
    elapsed = [result.elapsed_ms for result in results]
    colors = ["#4c78a8", "#72b7b2", "#f58518", "#54a24b"]

    ax_path = fig.add_subplot(layout[2, 0])
    ax_visited = fig.add_subplot(layout[2, 1])
    ax_time = fig.add_subplot(layout[2, 2])
    ax_summary = fig.add_subplot(layout[2, 3])

    ax_path.bar(names, path_steps, color=colors)
    ax_path.set_title("Path Length")
    ax_path.set_ylabel("steps")
    ax_path.grid(True, axis="y", alpha=0.25)

    ax_visited.bar(names, visited_counts, color=colors)
    ax_visited.set_title("Search Effort")
    ax_visited.set_ylabel("visited nodes")
    ax_visited.grid(True, axis="y", alpha=0.25)

    ax_time.bar(names, elapsed, color=colors)
    ax_time.set_title("Runtime")
    ax_time.set_ylabel("milliseconds")
    ax_time.grid(True, axis="y", alpha=0.25)

    ax_summary.axis("off")
    summary = (
        "DFS: not guaranteed shortest\n"
        "BFS: shortest on unweighted grids\n"
        "Dijkstra: shortest with nonnegative costs\n"
        "A*: shortest with an admissible heuristic\n\n"
        "Blue path = final route\n"
        "Light blue = searched cells"
    )
    ax_summary.text(0.0, 0.95, summary, va="top", fontsize=11)

    fig.suptitle("Path Finding Algorithms: DFS vs BFS vs Dijkstra vs A*", fontsize=15)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_results(results: list[SearchResult]) -> None:
    print("Path finding comparison")
    print("=======================")
    print(f"{'Algorithm':<10} {'Found':<7} {'Steps':>8} {'Visited':>10} {'Time(ms)':>10}")
    for result in results:
        print(
            f"{result.name:<10} {str(result.found):<7} "
            f"{result.path_steps:>8} {result.visited_count:>10} {result.elapsed_ms:>10.3f}"
        )


def main() -> None:
    current_dir = Path(__file__).resolve().parent
    output_path = current_dir / "outputs" / "pathfinding_comparison.png"

    grid, start, goal = make_demo_grid()
    results = run_all_algorithms(grid, start, goal)
    print_results(results)
    plot_results(grid, start, goal, results, output_path)
    print()
    print(f"Figure saved to: {output_path}")


if __name__ == "__main__":
    main()
