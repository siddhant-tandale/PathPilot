"""Reproducible benchmark: every planner on the same set of seeded maps.

Each (map style, density, seed) triple defines one problem. Every planner runs
on exactly that problem, and sampling planners use the map seed as their own
random seed, so the whole table regenerates identically.
"""

from __future__ import annotations

import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

from .grid import random_problem
from .planners import PLANNERS, run_planner

# (style, density) scenarios. "blocks" maps are the main comparison; the
# "noise" scenario is there to show where sampling planners struggle.
DEFAULT_SCENARIOS = [
    ("blocks", 0.15),
    ("blocks", 0.25),
    ("blocks", 0.35),
    ("noise", 0.25),
]

FIELDS = ["style", "density", "seed", "planner", "success", "cost",
          "cost_ratio", "nodes", "checks", "runtime_ms"]


@dataclass
class Row:
    style: str
    density: float
    seed: int
    planner: str
    success: bool
    cost: float
    cost_ratio: float  # cost / A* cost on the same map (nan if either failed)
    nodes: int
    checks: int
    runtime_ms: float


def run_benchmark(n_maps: int = 20, size: int = 50,
                  scenarios=DEFAULT_SCENARIOS, planners=None,
                  progress: bool = True) -> list[Row]:
    planners = planners or list(PLANNERS)
    rows: list[Row] = []
    for style, density in scenarios:
        for seed in range(n_maps):
            problem = random_problem(seed=seed, size=size, density=density,
                                     style=style)
            results = {}
            for key in planners:
                kwargs = {"seed": seed} if key in ("rrt", "rrt_star", "prm") else {}
                results[key] = run_planner(PLANNERS[key], problem, **kwargs)
            ref = results.get("astar")
            ref_cost = ref.cost if ref and ref.success else math.nan
            for r in results.values():
                ratio = r.cost / ref_cost if r.success and not math.isnan(ref_cost) else math.nan
                rows.append(Row(style, density, seed, r.planner, r.success,
                                r.cost, ratio, r.nodes, r.checks,
                                r.runtime_s * 1000))
            if progress:
                print(f"\r{style} {density:.2f}: map {seed + 1}/{n_maps}",
                      end="", flush=True)
        if progress:
            print()
    return rows


def save_csv(rows: list[Row], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(FIELDS)
        for r in rows:
            w.writerow([r.style, r.density, r.seed, r.planner, int(r.success),
                        f"{r.cost:.4f}", f"{r.cost_ratio:.4f}", r.nodes,
                        r.checks, f"{r.runtime_ms:.3f}"])


def load_csv(path: Path) -> list[Row]:
    rows = []
    with open(path) as f:
        for d in csv.DictReader(f):
            rows.append(Row(d["style"], float(d["density"]), int(d["seed"]),
                            d["planner"], d["success"] == "1", float(d["cost"]),
                            float(d["cost_ratio"]), int(d["nodes"]),
                            int(d["checks"]), float(d["runtime_ms"])))
    return rows


def _median(values):
    values = [v for v in values if not math.isnan(v)]
    return statistics.median(values) if values else math.nan


def summarize(rows: list[Row], style: str | None = None,
              density: float | None = None) -> list[dict]:
    """Per-planner summary over the selected rows, in first-seen planner order."""
    sel = [r for r in rows
           if (style is None or r.style == style)
           and (density is None or math.isclose(r.density, density))]
    order = list(dict.fromkeys(r.planner for r in sel))
    out = []
    for name in order:
        rs = [r for r in sel if r.planner == name]
        ok = [r for r in rs if r.success]
        out.append({
            "planner": name,
            "runs": len(rs),
            "success_rate": len(ok) / len(rs) if rs else math.nan,
            "cost_ratio": _median([r.cost_ratio for r in ok]),
            "runtime_ms": _median([r.runtime_ms for r in rs]),
            "checks": _median([float(r.checks) for r in rs]),
            "nodes": _median([float(r.nodes) for r in rs]),
        })
    return out


def markdown_table(summary: list[dict]) -> str:
    lines = [
        "| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for s in summary:
        ratio = "n/a" if math.isnan(s["cost_ratio"]) else f"{s['cost_ratio']:.3f}"
        lines.append(
            f"| {s['planner']} | {s['success_rate'] * 100:.0f}% | {ratio} | "
            f"{s['runtime_ms']:.1f} | {s['checks']:,.0f} | {s['nodes']:,.0f} |")
    return "\n".join(lines)


def summary_markdown(rows: list[Row]) -> str:
    """Full results section: one table per scenario plus an all-blocks table."""
    parts = ["## All blocks maps\n", markdown_table(summarize(rows, style="blocks")), ""]
    for style, density in dict.fromkeys((r.style, r.density) for r in rows):
        n = len({r.seed for r in rows if r.style == style and r.density == density})
        parts += [f"## {style}, {density:.0%} obstacles ({n} maps)\n",
                  markdown_table(summarize(rows, style, density)), ""]
    return "\n".join(parts)
