"""Graph search on the 8-connected grid: A*, weighted A*, and Dijkstra.

All three are the same best-first search with a different priority:

    f(n) = g(n) + w * h(n)

* Dijkstra: w = 0. Expands in order of cost-so-far. Optimal, but explores in
  every direction.
* A*: w = 1 with an admissible heuristic. Optimal, and the heuristic steers
  the search toward the goal.
* Weighted A*: w > 1. Expands fewer nodes, and the path is guaranteed to be
  within a factor w of optimal.
"""

from __future__ import annotations

import heapq
import itertools
import math

from ..grid import MOVES, Cell, Problem
from .base import PlanResult

SQRT2 = math.sqrt(2.0)


def octile(a: Cell, b: Cell) -> float:
    """Exact shortest-path length between two cells on an empty 8-connected grid.

    Move diagonally until aligned on one axis, then straight. It never
    overestimates the true cost (obstacles can only make paths longer), so it
    is admissible and A* stays optimal.
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (SQRT2 - 1.0) * min(dx, dy) + max(dx, dy)


def best_first_search(problem: Problem, weight: float = 1.0,
                      name: str = "A*") -> PlanResult:
    grid = problem.grid
    start, goal = problem.start_cell, problem.goal_cell

    if not (grid.is_free_cell(*start) and grid.is_free_cell(*goal)):
        return PlanResult(name, False, [], math.inf, 0)

    # Heap entries: (f, h, tiebreak, cell). Breaking f-ties on smaller h
    # prefers nodes closer to the goal, which noticeably cuts expansions on
    # open maps where many nodes share the same f.
    counter = itertools.count()
    h0 = octile(start, goal)
    open_heap = [(weight * h0, h0, next(counter), start)]
    g: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    closed: set[Cell] = set()
    expanded: list[Cell] = []

    while open_heap:
        _, _, _, current = heapq.heappop(open_heap)
        if current in closed:
            # Stale entry: this cell was already expanded with a lower cost.
            continue
        closed.add(current)
        expanded.append(current)

        if current == goal:
            cells = [current]
            while cells[-1] in parent:
                cells.append(parent[cells[-1]])
            cells.reverse()
            return PlanResult(
                name, True, [(x + 0.5, y + 0.5) for x, y in cells], 0.0,
                nodes=len(expanded),
                explored=[(x + 0.5, y + 0.5) for x, y in expanded],
            )

        cx, cy = current
        for dx, dy in MOVES:
            nb = (cx + dx, cy + dy)
            if nb in closed or not grid.can_move(cx, cy, dx, dy):
                continue
            new_g = g[current] + (SQRT2 if dx and dy else 1.0)
            if new_g < g.get(nb, math.inf):
                g[nb] = new_g
                parent[nb] = current
                h = octile(nb, goal)
                heapq.heappush(open_heap,
                               (new_g + weight * h, h, next(counter), nb))

    return PlanResult(name, False, [], math.inf, nodes=len(expanded),
                      explored=[(x + 0.5, y + 0.5) for x, y in expanded])


def astar(problem: Problem) -> PlanResult:
    return best_first_search(problem, weight=1.0, name="A*")


def dijkstra(problem: Problem) -> PlanResult:
    return best_first_search(problem, weight=0.0, name="Dijkstra")


def weighted_astar(problem: Problem, weight: float = 2.0) -> PlanResult:
    return best_first_search(problem, weight=weight, name=f"Weighted A* (w={weight:g})")
