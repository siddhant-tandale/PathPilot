import math

import numpy as np
import pytest

from pathpilot.grid import GridMap, Problem, random_problem
from pathpilot.planners import PLANNERS, SAMPLING, run_planner
from pathpilot.planners.grid_search import octile


def solve(key, problem, seed=0):
    kwargs = {"seed": seed} if key in SAMPLING else {}
    return run_planner(PLANNERS[key], problem, **kwargs)


def assert_valid_path(problem, result):
    assert result.success
    assert result.path[0] == pytest.approx(problem.start)
    assert result.path[-1] == pytest.approx(problem.goal)
    for a, b in zip(result.path, result.path[1:]):
        assert problem.grid.segment_free(a, b), f"segment {a}->{b} hits an obstacle"


@pytest.mark.parametrize("key", list(PLANNERS))
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_every_planner_returns_collision_free_path(key, seed):
    problem = random_problem(seed=seed, size=40, density=0.2)
    r = solve(key, problem, seed)
    assert_valid_path(problem, r)
    assert r.cost >= math.dist(problem.start, problem.goal) - 1e-9


def test_astar_matches_dijkstra_cost():
    for seed in range(10):
        p = random_problem(seed=seed, density=0.3)
        a = solve("astar", p)
        d = solve("dijkstra", p)
        assert a.cost == pytest.approx(d.cost)
        # The heuristic should never make A* expand more than Dijkstra.
        assert a.nodes <= d.nodes


def test_astar_on_empty_grid_matches_octile_distance():
    p = Problem(GridMap(np.zeros((30, 30))), (2.5, 3.5), (25.5, 11.5))
    r = solve("astar", p)
    assert r.cost == pytest.approx(octile(p.start_cell, p.goal_cell))


def test_weighted_astar_respects_suboptimality_bound():
    for seed in range(10):
        p = random_problem(seed=seed, density=0.3)
        opt = solve("astar", p).cost
        w = solve("wastar", p)
        assert opt - 1e-9 <= w.cost <= 2.0 * opt


def test_unreachable_goal_fails_cleanly():
    occ = np.zeros((20, 20), dtype=int)
    occ[:, 10] = 1  # wall splitting the map
    p = Problem(GridMap(occ), (2.5, 2.5), (17.5, 17.5))
    for key in ("astar", "dijkstra", "wastar"):
        r = solve(key, p)
        assert not r.success and r.path == [] and math.isinf(r.cost)
    for key in ("rrt", "prm"):
        r = run_planner(PLANNERS[key], p, seed=0,
                        **({"max_iters": 500} if key == "rrt" else {"n_samples": 150}))
        assert not r.success


def test_start_inside_obstacle_fails():
    occ = np.zeros((10, 10), dtype=int)
    occ[2, 2] = 1
    p = Problem(GridMap(occ), (2.5, 2.5), (8.5, 8.5))
    assert not solve("astar", p).success


def test_sampling_planners_are_reproducible():
    p = random_problem(seed=3, density=0.25)
    for key in SAMPLING:
        a, b = solve(key, p, seed=5), solve(key, p, seed=5)
        assert a.path == b.path


def test_rrt_star_cost_never_increases():
    p = random_problem(seed=4, density=0.25)
    r = solve("rrt_star", p, seed=4)
    costs = [c for _, c in r.extra["cost_history"] if math.isfinite(c)]
    assert costs, "RRT* never found a solution"
    assert all(b <= a + 1e-9 for a, b in zip(costs, costs[1:]))
    assert r.cost == pytest.approx(costs[-1])


def test_rrt_star_tree_costs_are_consistent_after_rewiring():
    """Every node's stored cost equals the length of its path to the root."""
    from pathpilot.planners.rrt import rrt_star
    p = random_problem(seed=6, density=0.25)
    r = rrt_star(p, max_iters=800, seed=6, snapshot_every=0)
    nodes = r.extra["nodes_array"]
    parent = r.extra["parent_array"]
    cost = r.extra["cost_array"]
    # Rewiring must propagate cost changes down each subtree. Recompute
    # cost-to-root from the parent pointers and compare.
    for i in range(1, len(parent)):
        length, k, steps = 0.0, i, 0
        while parent[k] != -1:
            length += float(np.linalg.norm(nodes[k] - nodes[parent[k]]))
            k = parent[k]
            steps += 1
            assert steps <= len(parent), "cycle in tree"
        assert k == 0
        assert cost[i] == pytest.approx(length, abs=1e-6)


def test_collision_checks_are_counted():
    p = random_problem(seed=0, density=0.2)
    for key in PLANNERS:
        assert solve(key, p).checks > 0
