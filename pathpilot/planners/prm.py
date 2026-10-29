"""Probabilistic Roadmap (Kavraki et al., 1996).

PRM splits planning into two phases:

1. Build: sample collision-free points and connect each to its k nearest
   neighbors with straight, collision-free edges. This roadmap only depends on
   the map, so in a real system it is built once and reused.
2. Query: connect the start and goal into the roadmap, then run graph search
   (Dijkstra here) over it.

This implementation builds a fresh roadmap per query so the benchmark charges
PRM for its full cost. ``PlanResult.extra["build_s"]`` and ``["query_s"]``
split the runtime so the reuse argument can still be made from the numbers.
"""

from __future__ import annotations

import heapq
import math
import time

import numpy as np

from ..grid import Problem
from .base import PlanResult


def prm(problem: Problem, n_samples: int = 600, k: int = 10,
        max_edge: float = 8.0, seed: int = 0) -> PlanResult:
    grid = problem.grid
    rng = np.random.default_rng(seed)
    t0 = time.perf_counter()

    # --- Build phase -------------------------------------------------------
    samples = []
    attempts = 0
    while len(samples) < n_samples and attempts < 50 * n_samples:
        attempts += 1
        p = (rng.uniform(0, grid.width), rng.uniform(0, grid.height))
        if grid.is_free_point(p):
            samples.append(p)

    # Index 0 is the start and index 1 is the goal; they are connected the
    # same way as any other node.
    pts = np.array([problem.start, problem.goal] + samples, dtype=float)
    m = len(pts)
    adj: list[list[tuple[int, float]]] = [[] for _ in range(m)]
    edges = []
    tested: set[tuple[int, int]] = set()

    # Pairwise distances. Fine for a few hundred nodes; a k-d tree would
    # replace this for thousands.
    diff = pts[:, None, :] - pts[None, :, :]
    dist = np.sqrt((diff ** 2).sum(axis=2))
    np.fill_diagonal(dist, np.inf)

    for i in range(m):
        for j in np.argsort(dist[i])[:k]:
            j = int(j)
            d = float(dist[i, j])
            if d > max_edge:
                break
            key = (min(i, j), max(i, j))
            if key in tested:
                continue
            tested.add(key)
            if grid.segment_free(tuple(pts[i]), tuple(pts[j])):
                adj[i].append((j, d))
                adj[j].append((i, d))
                edges.append((tuple(pts[i]), tuple(pts[j])))
    build_s = time.perf_counter() - t0

    # --- Query phase: Dijkstra from node 0 to node 1 ------------------------
    t1 = time.perf_counter()
    best = [math.inf] * m
    parent = [-1] * m
    best[0] = 0.0
    heap = [(0.0, 0)]
    expanded = 0
    while heap:
        c, u = heapq.heappop(heap)
        if c > best[u]:
            continue
        expanded += 1
        if u == 1:
            break
        for v, w in adj[u]:
            if c + w < best[v]:
                best[v] = c + w
                parent[v] = u
                heapq.heappush(heap, (c + w, v))
    query_s = time.perf_counter() - t1

    extra = {"build_s": build_s, "query_s": query_s,
             "roadmap_edges": len(edges), "query_expanded": expanded}
    if math.isinf(best[1]):
        return PlanResult("PRM", False, [], math.inf, nodes=m,
                          explored=edges, extra=extra)

    path_idx = [1]
    while path_idx[-1] != 0:
        path_idx.append(parent[path_idx[-1]])
    path_idx.reverse()
    path = [(float(pts[i, 0]), float(pts[i, 1])) for i in path_idx]
    return PlanResult("PRM", True, path, 0.0, nodes=m, explored=edges,
                      extra=extra)
