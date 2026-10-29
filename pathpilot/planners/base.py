"""Shared result type and timing wrapper for every planner."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field

from ..grid import Point, Problem, path_length


@dataclass
class PlanResult:
    """What every planner returns.

    Attributes:
        planner: Planner name.
        success: Whether a path from start to goal was found.
        path: Continuous waypoints from start to goal (empty on failure).
        cost: Euclidean length of ``path`` (``inf`` on failure).
        nodes: Nodes expanded (graph search) or tree / roadmap size (sampling).
        checks: Collision queries made against the map.
        runtime_s: Wall-clock planning time in seconds.
        explored: Search history for animation. For grid planners this is the
            list of expanded cell centers; for sampling planners it is the list
            of edges ``(parent_point, child_point)`` in the order they were added.
        extra: Planner-specific data (for example the RRT* cost history).
    """

    planner: str
    success: bool
    path: list[Point]
    cost: float
    nodes: int
    checks: int = 0
    runtime_s: float = 0.0
    explored: list = field(default_factory=list, repr=False)
    extra: dict = field(default_factory=dict, repr=False)


PlannerFn = Callable[..., PlanResult]


def run_planner(fn: PlannerFn, problem: Problem, **kwargs) -> PlanResult:
    """Run a planner, filling in timing, collision-check count, and path cost."""
    problem.grid.checks = 0
    t0 = time.perf_counter()
    result = fn(problem, **kwargs)
    result.runtime_s = time.perf_counter() - t0
    result.checks = problem.grid.checks
    result.cost = path_length(result.path) if result.success else float("inf")
    return result
