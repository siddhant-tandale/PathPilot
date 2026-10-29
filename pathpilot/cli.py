"""Command-line interface.

    python -m pathpilot plan --planner rrt_star --seed 3
    python -m pathpilot benchmark --maps 20
    python -m pathpilot animate --seed 4
"""

from __future__ import annotations

import argparse
from pathlib import Path

from .grid import random_problem
from .planners import PLANNERS, SAMPLING, run_planner


def _add_map_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--seed", type=int, default=0, help="map seed (also the planner seed)")
    p.add_argument("--size", type=int, default=50, help="map is size x size cells")
    p.add_argument("--density", type=float, default=0.25, help="obstacle fraction")
    p.add_argument("--style", choices=["blocks", "noise"], default="blocks")


def cmd_plan(args) -> None:
    from .viz import plot_result

    problem = random_problem(args.seed, args.size, args.density, args.style)
    kwargs = {"seed": args.seed} if args.planner in SAMPLING else {}
    r = run_planner(PLANNERS[args.planner], problem, **kwargs)
    status = f"cost {r.cost:.2f}" if r.success else "FAILED"
    print(f"{r.planner}: {status} | {r.runtime_s * 1000:.1f} ms | "
          f"{r.nodes} nodes | {r.checks} collision checks")
    out = Path(args.out or f"{args.planner}_seed{args.seed}.png")
    plot_result(problem, r, out)
    print(f"saved {out}")


def cmd_benchmark(args) -> None:
    from . import benchmark as bm
    from .planners.rrt import rrt_star
    from .viz import plot_benchmark, plot_convergence

    out = Path(args.out)
    figs = Path(args.figures)
    out.mkdir(parents=True, exist_ok=True)
    figs.mkdir(parents=True, exist_ok=True)

    rows = bm.run_benchmark(n_maps=args.maps, size=args.size)
    bm.save_csv(rows, out / "benchmark.csv")
    (out / "summary.md").write_text(
        f"# Benchmark results\n\n{args.maps} maps per scenario, {args.size}x{args.size} "
        "cells. Path cost vs A* is the median ratio over maps where both "
        "succeeded. Runtime, checks, and nodes are medians over all runs.\n\n"
        + bm.summary_markdown(rows))
    print(f"saved {out / 'benchmark.csv'} and {out / 'summary.md'}")

    blocks = bm.summarize(rows, style="blocks")
    n_blocks = len({(r.density, r.seed) for r in rows if r.style == "blocks"})
    plot_benchmark(blocks, figs / "benchmark.png",
                   f"Six planners on {n_blocks} random {args.size}x{args.size} maps (medians)")
    noise = bm.summarize(rows, style="noise")
    if noise:
        n_noise = len({r.seed for r in rows if r.style == "noise"})
        plot_benchmark(noise, figs / "benchmark_noise.png",
                       f"Same planners on {n_noise} salt-and-pepper maps (narrow gaps everywhere)")

    # RRT* convergence on the blocks maps, relative to A* on the same map.
    histories, refs = [], []
    astar_cost = {(r.density, r.seed): r.cost for r in rows
                  if r.style == "blocks" and r.planner == "A*" and r.success}
    for (density, seed), ref in astar_cost.items():
        p = random_problem(seed, args.size, density, "blocks")
        res = rrt_star(p, seed=seed, snapshot_every=0)
        histories.append(res.extra["cost_history"])
        refs.append(ref)
    plot_convergence(histories, refs, figs / "rrt_star_convergence.png",
                     f"RRT* keeps shortening its path ({len(refs)} maps)")
    print(f"saved figures to {figs}/")


def cmd_animate(args) -> None:
    from .viz import animate

    problem = random_problem(args.seed, args.size, args.density, args.style)
    results = []
    for key in PLANNERS:
        kwargs = {"seed": args.seed} if key in SAMPLING else {}
        results.append(run_planner(PLANNERS[key], problem, **kwargs))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    animate(problem, results, out)
    print(f"saved {out}")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(prog="pathpilot", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("plan", help="run one planner on one map and save a figure")
    p.add_argument("--planner", choices=list(PLANNERS), default="astar")
    p.add_argument("--out", default=None)
    _add_map_args(p)
    p.set_defaults(func=cmd_plan)

    b = sub.add_parser("benchmark", help="run every planner on many maps")
    b.add_argument("--maps", type=int, default=20, help="maps per scenario")
    b.add_argument("--size", type=int, default=50)
    b.add_argument("--out", default="results")
    b.add_argument("--figures", default="docs")
    b.set_defaults(func=cmd_benchmark)

    a = sub.add_parser("animate", help="side-by-side GIF of all planners on one map")
    a.add_argument("--out", default="docs/demo.gif")
    _add_map_args(a)
    a.set_defaults(func=cmd_animate)

    args = parser.parse_args(argv)
    args.func(args)
