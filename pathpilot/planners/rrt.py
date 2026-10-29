"""Rapidly-exploring Random Trees: RRT and RRT*.

RRT (LaValle, 1998) grows a tree from the start by repeatedly sampling a
random point and extending the nearest tree node a fixed step toward it. It
finds *a* path quickly, but the path is jagged and the cost does not improve.

RRT* (Karaman and Frazzoli, 2011) adds two steps per new node:

1. Choose parent: among nearby nodes, connect through whichever gives the
   new node the lowest cost-from-start, not just the nearest one.
2. Rewire: for each nearby node, if going through the new node is cheaper,
   switch its parent to the new node.

With the neighborhood radius shrinking like (log n / n)^(1/d), the path cost
converges toward the optimum as the number of samples grows (asymptotic
optimality). That is why RRT* keeps running after its first solution.
"""

from __future__ import annotations

import math

import numpy as np

from ..grid import Point, Problem
from .base import PlanResult


def _sample(rng: np.random.Generator, problem: Problem, goal_bias: float) -> np.ndarray:
    """Uniform sample over the map, or the goal itself with probability goal_bias."""
    if rng.random() < goal_bias:
        return np.asarray(problem.goal, dtype=float)
    return np.array([rng.uniform(0, problem.grid.width),
                     rng.uniform(0, problem.grid.height)])


def _steer(a: np.ndarray, b: np.ndarray, step: float) -> np.ndarray:
    """Move from a toward b by at most ``step``."""
    d = b - a
    dist = math.hypot(d[0], d[1])
    if dist <= step:
        return b.copy()
    return a + d * (step / dist)


def _trace(nodes: np.ndarray, parent: np.ndarray, idx: int) -> list[Point]:
    path = []
    while idx != -1:
        path.append((float(nodes[idx, 0]), float(nodes[idx, 1])))
        idx = int(parent[idx])
    path.reverse()
    return path


def rrt(problem: Problem, max_iters: int = 5000, step: float = 2.0,
        goal_bias: float = 0.05, seed: int = 0) -> PlanResult:
    """Basic RRT. Returns as soon as the tree connects to the goal."""
    grid = problem.grid
    rng = np.random.default_rng(seed)
    start = np.asarray(problem.start, dtype=float)
    goal = np.asarray(problem.goal, dtype=float)

    nodes = np.empty((max_iters + 2, 2))
    parent = np.full(max_iters + 2, -1, dtype=int)
    nodes[0] = start
    n = 1
    edges = []

    for it in range(max_iters):
        q = _sample(rng, problem, goal_bias)
        if not grid.is_free_point(q):
            continue
        # Nearest neighbor by brute force. O(n) per query, but vectorized, so
        # it is fast enough for a few thousand nodes. A k-d tree would be the
        # next step for larger problems.
        i = int(np.argmin(((nodes[:n] - q) ** 2).sum(axis=1)))
        new = _steer(nodes[i], q, step)
        if not grid.segment_free(nodes[i], new):
            continue
        nodes[n], parent[n] = new, i
        edges.append((tuple(nodes[i]), tuple(new)))
        n += 1

        if np.linalg.norm(goal - new) <= step and grid.segment_free(new, goal):
            nodes[n], parent[n] = goal, n - 1
            edges.append((tuple(new), tuple(goal)))
            n += 1
            return PlanResult("RRT", True, _trace(nodes, parent, n - 1), 0.0,
                              nodes=n, explored=edges,
                              extra={"iterations": it + 1})

    return PlanResult("RRT", False, [], math.inf, nodes=n, explored=edges,
                      extra={"iterations": max_iters})


def rrt_star(problem: Problem, max_iters: int = 3000, step: float = 2.0,
             goal_bias: float = 0.05, max_radius: float = 5.0, seed: int = 0,
             snapshot_every: int = 50) -> PlanResult:
    """RRT* with choose-parent and rewiring. Runs all ``max_iters`` iterations."""
    grid = problem.grid
    rng = np.random.default_rng(seed)
    start = np.asarray(problem.start, dtype=float)
    goal = np.asarray(problem.goal, dtype=float)

    cap = max_iters + 1
    nodes = np.empty((cap, 2))
    parent = np.full(cap, -1, dtype=int)
    cost = np.zeros(cap)
    children: list[list[int]] = [[] for _ in range(cap)]
    nodes[0] = start
    n = 1

    # Neighborhood radius r(n) = gamma * sqrt(log n / n), from Karaman and
    # Frazzoli. In 2D, gamma must exceed 2 * sqrt(1.5) * sqrt(free_area / pi)
    # for asymptotic optimality. The radius is capped at max_radius so early
    # iterations do not check huge neighborhoods.
    free_area = grid.free_fraction() * grid.width * grid.height
    gamma = 2.0 * math.sqrt(1.5) * math.sqrt(free_area / math.pi) * 1.1

    # Nodes verified to have a collision-free straight line to the goal.
    # Their costs keep changing as the tree is rewired, so the best goal
    # connection is recomputed from the current cost array each time.
    goal_links: dict[int, float] = {}
    cost_history: list[tuple[int, float]] = []
    snapshots: list[np.ndarray] = []
    first_solution_iter = None

    def best_goal() -> tuple[int, float]:
        if not goal_links:
            return -1, math.inf
        j = min(goal_links, key=lambda k: cost[k] + goal_links[k])
        return j, float(cost[j] + goal_links[j])

    for it in range(max_iters):
        q = _sample(rng, problem, goal_bias)
        if grid.is_free_point(q):
            d2_all = ((nodes[:n] - q) ** 2).sum(axis=1)
            i = int(np.argmin(d2_all))
            new = _steer(nodes[i], q, step)
            if grid.segment_free(nodes[i], new):
                r = min(gamma * math.sqrt(math.log(n + 1) / (n + 1)), max_radius)
                d_new = np.sqrt(((nodes[:n] - new) ** 2).sum(axis=1))
                near = np.flatnonzero(d_new <= r)

                # Choose parent: try neighbors in order of the cost they would
                # give the new node, and take the first collision-free one.
                # Sorting first means most calls do one or two collision checks.
                best_p = i
                best_c = cost[i] + d_new[i]
                if near.size:
                    cand = cost[near] + d_new[near]
                    for k in np.argsort(cand):
                        j = int(near[k])
                        if cand[k] >= best_c:
                            break
                        if grid.segment_free(nodes[j], new):
                            best_p, best_c = j, float(cand[k])
                            break

                idx = n
                nodes[idx], parent[idx], cost[idx] = new, best_p, best_c
                children[best_p].append(idx)
                n += 1

                # Rewire: reroute neighbors through the new node if cheaper.
                for j in near:
                    j = int(j)
                    if j == best_p:
                        continue
                    via = best_c + d_new[j]
                    if via + 1e-9 < cost[j] and grid.segment_free(new, nodes[j]):
                        children[parent[j]].remove(j)
                        parent[j] = idx
                        children[idx].append(j)
                        # Propagate the cost decrease to j's whole subtree.
                        delta = cost[j] - via
                        stack = [j]
                        while stack:
                            k = stack.pop()
                            cost[k] -= delta
                            stack.extend(children[k])

                d_goal = float(np.linalg.norm(goal - new))
                if d_goal <= max_radius and grid.segment_free(new, goal):
                    goal_links[idx] = d_goal
                    if first_solution_iter is None:
                        first_solution_iter = it + 1

        if (it + 1) % 10 == 0:
            cost_history.append((it + 1, best_goal()[1]))
        if snapshot_every and (it + 1) % snapshot_every == 0:
            snapshots.append(parent[:n].copy())

    edges = [(tuple(nodes[parent[k]]), tuple(nodes[k])) for k in range(1, n)]
    extra = {
        "iterations": max_iters,
        "first_solution_iter": first_solution_iter,
        "cost_history": cost_history,
        "snapshots": snapshots,
        "nodes_array": nodes[:n].copy(),
        "parent_array": parent[:n].copy(),
        "cost_array": cost[:n].copy(),
    }
    j, _ = best_goal()
    if j < 0:
        return PlanResult("RRT*", False, [], math.inf, nodes=n,
                          explored=edges, extra=extra)
    path = _trace(nodes, parent, j) + [(float(goal[0]), float(goal[1]))]
    return PlanResult("RRT*", True, path, 0.0, nodes=n, explored=edges,
                      extra=extra)
