"""Theta*: any-angle path planning on a grid (Nash, Daniel, Koenig, Felner, 2007).

A* on an 8-connected grid can only move horizontally, vertically, or at 45
degrees, so its "optimal" path zig-zags even through open space. Theta* keeps
the A* search but changes how a node gets its parent.

When expanding cell s and looking at neighbor n, Theta* first asks: can n see
s's *parent* in a straight line?

* Yes (path 2): connect n straight to parent(s), skipping s entirely.
  g(n) = g(parent(s)) + |parent(s) - n|
* No (path 1): fall back to the ordinary A* update through s.
  g(n) = g(s) + |s - n|

Parents can therefore be any earlier cell in view, not just a neighbor, so
paths bend only at obstacle corners. Theta* is not guaranteed to find the true
shortest any-angle path, but it is usually very close, at a cost of one
line-of-sight check per neighbor.

The heuristic changes too. Octile distance assumes 8 directions and can
overestimate an any-angle path, so Theta* uses straight-line distance, which
never does.
"""

from __future__ import annotations

import heapq
import itertools
import math

from ..grid import MOVES, Cell, Problem
from .base import PlanResult


def _center(c: Cell) -> tuple[float, float]:
    return (c[0] + 0.5, c[1] + 0.5)


def _dist(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def theta_star(problem: Problem) -> PlanResult:
    grid = problem.grid
    start, goal = problem.start_cell, problem.goal_cell
    name = "Theta*"

    if not (grid.is_free_cell(*start) and grid.is_free_cell(*goal)):
        return PlanResult(name, False, [], math.inf, 0)

    counter = itertools.count()
    h0 = _dist(start, goal)
    open_heap = [(h0, h0, next(counter), start)]
    g: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell] = {start: start}  # the start is its own parent
    closed: set[Cell] = set()
    expanded: list[Cell] = []

    while open_heap:
        _, _, _, s = heapq.heappop(open_heap)
        if s in closed:
            continue
        closed.add(s)
        expanded.append(s)

        if s == goal:
            cells = [s]
            while parent[cells[-1]] != cells[-1]:
                cells.append(parent[cells[-1]])
            cells.reverse()
            return PlanResult(name, True, [_center(c) for c in cells], 0.0,
                              nodes=len(expanded),
                              explored=[_center(c) for c in expanded])

        sx, sy = s
        ps = parent[s]
        for dx, dy in MOVES:
            n = (sx + dx, sy + dy)
            if n in closed or not grid.can_move(sx, sy, dx, dy):
                continue
            # Path 2: straight line from s's parent, if it is visible.
            if ps != s and grid.segment_free(_center(ps), _center(n)):
                cand_parent, cand_g = ps, g[ps] + _dist(ps, n)
            # Path 1: the ordinary A* step through s.
            else:
                cand_parent, cand_g = s, g[s] + _dist(s, n)
            if cand_g < g.get(n, math.inf):
                g[n] = cand_g
                parent[n] = cand_parent
                h = _dist(n, goal)
                heapq.heappush(open_heap, (cand_g + h, h, next(counter), n))

    return PlanResult(name, False, [], math.inf, nodes=len(expanded),
                      explored=[_center(c) for c in expanded])
