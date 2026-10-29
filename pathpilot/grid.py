"""Occupancy grid maps, random map generation, and collision checking.

Coordinate convention
---------------------
* The grid is a 2D array indexed ``grid[row, col]`` = ``grid[y, x]``.
* ``1`` means occupied, ``0`` means free.
* Cell ``(x, y)`` covers the unit square ``[x, x+1) x [y, y+1)`` in continuous
  space, so a continuous point ``(px, py)`` lies in cell ``(floor(px), floor(py))``.
* Grid planners work on integer cells. Sampling planners work on continuous
  points. Both report paths as continuous points (grid paths use cell centers),
  so their costs are directly comparable.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

Point = tuple[float, float]
Cell = tuple[int, int]

# 8-connected moves. The first four are orthogonal, the last four diagonal.
MOVES: tuple[Cell, ...] = (
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
)


class GridMap:
    """A binary occupancy grid with collision queries.

    Every query increments ``self.checks`` so planners can be compared on how
    much collision checking they do, which is usually the dominant cost in a
    real planner, independent of how fast Python happens to be.
    """

    def __init__(self, occupancy: np.ndarray):
        occ = np.asarray(occupancy)
        if occ.ndim != 2:
            raise ValueError("occupancy grid must be 2D")
        self.occ = occ.astype(bool)
        self.height, self.width = self.occ.shape
        self.checks = 0

    # ------------------------------------------------------------------ cells
    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free_cell(self, x: int, y: int) -> bool:
        """True if cell (x, y) is inside the map and not an obstacle."""
        self.checks += 1
        return self.in_bounds(x, y) and not self.occ[y, x]

    def can_move(self, x: int, y: int, dx: int, dy: int) -> bool:
        """True if an 8-connected move from (x, y) by (dx, dy) is collision free.

        Diagonal moves are not allowed to cut obstacle corners: both of the
        orthogonal cells the diagonal passes between must be free. Without this
        rule a robot would squeeze through a gap of zero width.
        """
        if not self.is_free_cell(x + dx, y + dy):
            return False
        if dx != 0 and dy != 0:
            return self.is_free_cell(x + dx, y) and self.is_free_cell(x, y + dy)
        return True

    # ------------------------------------------------------- continuous space
    def is_free_point(self, p: Point) -> bool:
        return self.is_free_cell(math.floor(p[0]), math.floor(p[1]))

    def segment_free(self, p0: Point, p1: Point) -> bool:
        """Exact check that the straight segment p0 -> p1 crosses no obstacle.

        Uses grid traversal (Amanatides and Woo, 1987): walk the cells the
        segment passes through in order, one boundary crossing at a time,
        instead of sampling points along the segment. Sampling can step over a
        thin obstacle; traversal cannot.

        When the segment passes exactly through a cell corner, both cells that
        share that corner are checked, so the segment cannot slip diagonally
        between two touching obstacles.
        """
        self.checks += 1
        x0, y0 = float(p0[0]), float(p0[1])
        x1, y1 = float(p1[0]), float(p1[1])
        cx, cy = math.floor(x0), math.floor(y0)
        ex, ey = math.floor(x1), math.floor(y1)

        if not self._free_nocount(cx, cy):
            return False

        dx, dy = x1 - x0, y1 - y0
        step_x = (dx > 0) - (dx < 0)
        step_y = (dy > 0) - (dy < 0)

        # t is the fraction of the way along the segment (0 at p0, 1 at p1).
        # t_max_*: value of t at the next vertical / horizontal cell boundary.
        # t_delta_*: change in t between consecutive boundaries.
        if step_x > 0:
            t_max_x, t_delta_x = (cx + 1 - x0) / dx, 1 / dx
        elif step_x < 0:
            t_max_x, t_delta_x = (x0 - cx) / -dx, 1 / -dx
        else:
            t_max_x, t_delta_x = math.inf, math.inf
        if step_y > 0:
            t_max_y, t_delta_y = (cy + 1 - y0) / dy, 1 / dy
        elif step_y < 0:
            t_max_y, t_delta_y = (y0 - cy) / -dy, 1 / -dy
        else:
            t_max_y, t_delta_y = math.inf, math.inf

        eps = 1e-12
        # Upper bound on cells visited; guards against float edge cases.
        for _ in range(abs(ex - cx) + abs(ey - cy) + 2):
            if (cx, cy) == (ex, ey) or min(t_max_x, t_max_y) > 1:
                return True
            if abs(t_max_x - t_max_y) < eps:
                # Passing exactly through a corner: both side cells must be free.
                if not (self._free_nocount(cx + step_x, cy)
                        and self._free_nocount(cx, cy + step_y)):
                    return False
                cx += step_x
                cy += step_y
                t_max_x += t_delta_x
                t_max_y += t_delta_y
            elif t_max_x < t_max_y:
                cx += step_x
                t_max_x += t_delta_x
            else:
                cy += step_y
                t_max_y += t_delta_y
            if not self._free_nocount(cx, cy):
                return False
        return True

    def _free_nocount(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and not self.occ[y, x]

    # --------------------------------------------------------------- helpers
    def free_fraction(self) -> float:
        return float(1.0 - self.occ.mean())

    def inflate(self, radius: float) -> GridMap:
        """Return a copy with obstacles grown by ``radius`` cells.

        Planning on the inflated map for a point robot is equivalent to
        planning for a disk robot of that radius on the original map (the
        configuration space idea).
        """
        r = int(math.ceil(radius))
        out = self.occ.copy()
        padded = np.pad(self.occ, r, constant_values=False)
        for oy in range(-r, r + 1):
            for ox in range(-r, r + 1):
                if ox * ox + oy * oy > radius * radius:
                    continue
                out |= padded[r + oy: r + oy + self.height,
                              r + ox: r + ox + self.width]
        return GridMap(out)

    def reachable(self, start: Cell, goal: Cell) -> bool:
        """Breadth-first flood fill using the same move rules as the planners."""
        if not (self._free_nocount(*start) and self._free_nocount(*goal)):
            return False
        seen = {start}
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            if (x, y) == goal:
                return True
            for dx, dy in MOVES:
                n = (x + dx, y + dy)
                if n in seen or not self._free_nocount(*n):
                    continue
                if dx and dy and not (self._free_nocount(x + dx, y)
                                      and self._free_nocount(x, y + dy)):
                    continue
                seen.add(n)
                queue.append(n)
        return False


@dataclass
class Problem:
    """A planning query: a map plus continuous start and goal points."""

    grid: GridMap
    start: Point
    goal: Point
    seed: int | None = None
    style: str = ""
    density: float = 0.0

    @property
    def start_cell(self) -> Cell:
        return (math.floor(self.start[0]), math.floor(self.start[1]))

    @property
    def goal_cell(self) -> Cell:
        return (math.floor(self.goal[0]), math.floor(self.goal[1]))


def generate_occupancy(size: int, density: float, style: str,
                       rng: np.random.Generator) -> np.ndarray:
    """Random occupancy grid.

    ``style="blocks"`` drops random rectangles until the target density is
    reached, which looks like buildings or shelving. ``style="noise"`` marks
    each cell occupied independently, which is the classic salt-and-pepper map
    and is much harder for sampling planners (lots of narrow gaps).
    """
    if size < 8:
        raise ValueError("size must be at least 8")
    if not 0.0 <= density < 0.9:
        raise ValueError("density must be in [0, 0.9)")

    if style == "noise":
        return rng.random((size, size)) < density
    if style != "blocks":
        raise ValueError(f"unknown map style {style!r}")

    occ = np.zeros((size, size), dtype=bool)
    max_side = max(3, size // 6)
    while occ.mean() < density:
        w, h = rng.integers(2, max_side + 1, size=2)
        x, y = rng.integers(0, size - 1, size=2)
        occ[y:y + h, x:x + w] = True
    return occ


def random_problem(seed: int, size: int = 50, density: float = 0.25,
                   style: str = "blocks", margin: int = 2,
                   clearance: float | None = None,
                   max_tries: int = 200) -> Problem:
    """Generate a solvable problem with start near one corner, goal near the other.

    The same seed always produces the same map, so every planner in a
    benchmark sees exactly the same problems.

    ``clearance`` requires a path to exist even after obstacles are grown by
    that many cells, so the map is not only solvable through a 1-cell slot.
    It defaults to 1 for "blocks" maps (every map has a route at least 3
    cells wide) and 0 for "noise" maps, which are meant to be the
    narrow-passage stress test.
    """
    if clearance is None:
        clearance = 1.0 if style == "blocks" else 0.0
    rng = np.random.default_rng(seed)
    start = (margin + 0.5, margin + 0.5)
    goal = (size - margin - 0.5, size - margin - 0.5)
    for _ in range(max_tries):
        occ = generate_occupancy(size, density, style, rng)
        # Keep a small free patch around start and goal.
        for px, py in (start, goal):
            cx, cy = int(px), int(py)
            occ[max(cy - margin, 0): cy + margin + 1,
                max(cx - margin, 0): cx + margin + 1] = False
        grid = GridMap(occ)
        check = grid.inflate(clearance) if clearance > 0 else grid
        if check.reachable((int(start[0]), int(start[1])),
                           (int(goal[0]), int(goal[1]))):
            return Problem(grid, start, goal, seed=seed, style=style,
                           density=density)
    raise RuntimeError(f"no solvable map after {max_tries} tries (seed={seed})")


def path_length(path: list[Point]) -> float:
    """Total Euclidean length of a polyline."""
    if len(path) < 2:
        return 0.0
    arr = np.asarray(path, dtype=float)
    return float(np.linalg.norm(np.diff(arr, axis=0), axis=1).sum())
