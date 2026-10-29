import math

import numpy as np
import pytest

from pathpilot.grid import GridMap, path_length, random_problem


def empty(n=10):
    return GridMap(np.zeros((n, n), dtype=int))


def dense_segment_free(grid, p0, p1, step=1e-3):
    """Reference check: sample very finely along the segment."""
    n = int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / step) + 2
    for t in np.linspace(0, 1, n):
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        if not grid._free_nocount(math.floor(x), math.floor(y)):
            return False
    return True


def test_segment_on_empty_map_is_free():
    g = empty()
    assert g.segment_free((0.5, 0.5), (9.5, 9.5))
    assert g.segment_free((9.2, 0.1), (0.3, 7.7))


def test_segment_blocked_by_wall():
    occ = np.zeros((10, 10), dtype=int)
    occ[:, 5] = 1  # vertical wall at x = 5
    g = GridMap(occ)
    assert not g.segment_free((1.5, 1.5), (8.5, 2.5))
    assert g.segment_free((1.5, 1.5), (4.5, 8.5))


def test_segment_cannot_slip_between_diagonal_obstacles():
    # Two obstacles touching only at a corner; the diagonal line passes
    # exactly through that corner and must be rejected.
    occ = np.zeros((4, 4), dtype=int)
    occ[1, 2] = 1
    occ[2, 1] = 1
    g = GridMap(occ)
    assert not g.segment_free((1.5, 1.5), (2.5, 2.5))


def test_segment_matches_dense_sampling_on_random_maps():
    rng = np.random.default_rng(0)
    for _ in range(20):
        g = GridMap(rng.random((20, 20)) < 0.2)
        for _ in range(100):
            p0 = tuple(rng.uniform(0, 20, 2))
            p1 = tuple(rng.uniform(0, 20, 2))
            if not (g._free_nocount(*map(math.floor, p0))):
                continue
            exact = g.segment_free(p0, p1)
            ref = dense_segment_free(g, p0, p1)
            # Traversal is exact; dense sampling can only miss obstacles.
            # So: anything sampling says is blocked, traversal must also block.
            if not ref:
                assert not exact


def test_diagonal_move_cannot_cut_corners():
    occ = np.zeros((3, 3), dtype=int)
    occ[0, 1] = 1  # cell (x=1, y=0)
    g = GridMap(occ)
    assert not g.can_move(0, 0, 1, 1)
    assert g.can_move(0, 0, 0, 1)


def test_inflate_grows_obstacles():
    occ = np.zeros((7, 7), dtype=int)
    occ[3, 3] = 1
    g = GridMap(occ).inflate(1)
    assert g.occ[2:5, 3].all() and g.occ[3, 2:5].all()
    assert not g.occ[0, 0]


def test_random_problem_is_deterministic_and_solvable():
    a = random_problem(seed=7, density=0.3)
    b = random_problem(seed=7, density=0.3)
    assert np.array_equal(a.grid.occ, b.grid.occ)
    assert a.grid.reachable(a.start_cell, a.goal_cell)


@pytest.mark.parametrize("style", ["blocks", "noise"])
def test_density_is_close_to_target(style):
    p = random_problem(seed=1, size=60, density=0.25, style=style)
    assert 0.15 < 1 - p.grid.free_fraction() < 0.35


def test_path_length():
    assert path_length([(0, 0), (3, 4)]) == pytest.approx(5.0)
    assert path_length([(0, 0)]) == 0.0
