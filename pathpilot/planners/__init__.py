"""Planner registry.

Every planner takes a ``Problem`` (plus optional keyword settings) and returns
a ``PlanResult``. Use ``run_planner`` to also get timing and collision-check
counts filled in.
"""

from .base import PlanResult, run_planner
from .grid_search import astar, dijkstra, weighted_astar
from .prm import prm
from .rrt import rrt, rrt_star

PLANNERS = {
    "astar": astar,
    "dijkstra": dijkstra,
    "wastar": weighted_astar,
    "rrt": rrt,
    "rrt_star": rrt_star,
    "prm": prm,
}

# Planners whose output depends on a random seed.
SAMPLING = {"rrt", "rrt_star", "prm"}

__all__ = ["PLANNERS", "SAMPLING", "PlanResult", "run_planner",
           "astar", "dijkstra", "weighted_astar", "rrt", "rrt_star", "prm"]
