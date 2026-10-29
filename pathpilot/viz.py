"""Plots and animations: single plans, benchmark charts, RRT* convergence, demo GIF."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # render to files; works headless in CI

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import ListedColormap, to_rgba  # noqa: E402

from .grid import Problem  # noqa: E402
from .planners import PlanResult  # noqa: E402

# Fixed color per planner, in a colorblind-checked categorical order. A
# planner keeps its color in every figure.
COLORS = {
    "A*": "#2a78d6",
    "Dijkstra": "#eb6834",
    "Weighted A* (w=2)": "#1baf7a",
    "RRT": "#eda100",
    "RRT*": "#e87ba4",
    "PRM": "#008300",
}
SURFACE = "#fcfcfb"
OBSTACLE = "#3a3a38"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID_LINE = "#e4e3df"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.edgecolor": GRID_LINE,
    "axes.labelcolor": TEXT_2,
    "axes.titlecolor": TEXT,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "xtick.color": TEXT_2,
    "ytick.color": TEXT_2,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
})


def color_for(name: str) -> str:
    return COLORS.get(name, "#2a78d6")


# --------------------------------------------------------------- map drawing
def draw_map(ax, problem: Problem) -> None:
    g = problem.grid
    ax.imshow(g.occ.astype(int), cmap=ListedColormap([SURFACE, OBSTACLE]),
              origin="lower", extent=(0, g.width, 0, g.height),
              interpolation="nearest", vmin=0, vmax=1)
    ax.set_xlim(0, g.width)
    ax.set_ylim(0, g.height)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect("equal")
    for s in ax.spines.values():
        s.set_visible(False)


def draw_endpoints(ax, problem: Problem) -> None:
    ax.plot(*problem.start, "o", ms=9, mfc="white", mec=TEXT, mew=2, zorder=6)
    ax.plot(*problem.goal, "*", ms=15, mfc="white", mec=TEXT, mew=1.5, zorder=6)


def draw_path(ax, path, color, lw=2.5) -> None:
    if path:
        xs, ys = zip(*path)
        # White underlay keeps the path readable over explored regions.
        ax.plot(xs, ys, color=SURFACE, lw=lw + 2.5, zorder=4,
                solid_capstyle="round", solid_joinstyle="round")
        ax.plot(xs, ys, color=color, lw=lw, zorder=5,
                solid_capstyle="round", solid_joinstyle="round")


def draw_explored(ax, result: PlanResult, upto: int | None = None) -> None:
    color = color_for(result.planner)
    items = result.explored if upto is None else result.explored[:upto]
    if not items:
        return
    if isinstance(items[0][0], tuple):  # edges
        lc = LineCollection(items, colors=[to_rgba(color, 0.55)], linewidths=0.8,
                            zorder=3)
        ax.add_collection(lc)
    else:  # expanded cell centers
        pts = np.asarray(items)
        ax.scatter(pts[:, 0], pts[:, 1], s=6, marker="s", lw=0,
                   color=to_rgba(color, 0.35), zorder=3)


def plot_result(problem: Problem, result: PlanResult, out: Path | None = None):
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_map(ax, problem)
    draw_explored(ax, result)
    draw_path(ax, result.path, color_for(result.planner))
    draw_endpoints(ax, problem)
    status = f"cost {result.cost:.2f}" if result.success else "no path found"
    ax.set_title(f"{result.planner}: {status}, {result.runtime_s * 1000:.1f} ms",
                 loc="left")
    fig.tight_layout()
    if out:
        fig.savefig(out, dpi=150)
        plt.close(fig)
    return fig


# ------------------------------------------------------------ benchmark chart
def plot_benchmark(summary: list[dict], out: Path, title: str) -> None:
    """Four aligned panels, one row per planner: success, cost, runtime, checks.

    Success rate is a bar (it has a true zero). The other three are dot plots,
    because cost ratio clusters near 1.0 and runtime / checks span orders of
    magnitude on a log axis, where bars have no meaningful baseline.
    """
    names = [s["planner"] for s in summary][::-1]  # top-to-bottom = table order
    rows = {s["planner"]: s for s in summary}
    colors = [color_for(n) for n in names]
    y = np.arange(len(names))

    fig, axes = plt.subplots(1, 4, figsize=(13, 3.6), sharey=True,
                             gridspec_kw={"wspace": 0.18})
    panels = [
        ("success_rate", "Success rate", lambda v: f"{v * 100:.0f}%"),
        ("cost_ratio", "Path cost / A* cost", lambda v: f"{v:.3f}"),
        ("runtime_ms", "Runtime, ms (log)", lambda v: f"{v:.1f}" if v < 100 else f"{v:,.0f}"),
        ("checks", "Collision checks (log)", lambda v: f"{v:,.0f}"),
    ]
    for ax, (key, label, fmt) in zip(axes, panels):
        vals = np.array([rows[n][key] for n in names], dtype=float)
        ax.set_title(label, loc="left", fontsize=10)
        ax.grid(axis="x", color=GRID_LINE, lw=0.8)
        ax.set_axisbelow(True)
        for s in ("top", "right", "left"):
            ax.spines[s].set_visible(False)
        ax.tick_params(axis="y", length=0)

        if key == "success_rate":
            ax.barh(y, vals, height=0.6, color=colors, edgecolor=SURFACE, lw=2)
            ax.set_xlim(0, 1.18)
            ax.set_xticks([0, 0.5, 1])
            ax.set_xticklabels(["0%", "50%", "100%"])
            for yi, v in zip(y, vals):
                ax.text(v + 0.03, yi, fmt(v), va="center", fontsize=9, color=TEXT)
            continue

        ok = ~np.isnan(vals)
        ax.scatter(vals[ok], y[ok], s=70, c=[c for c, o in zip(colors, ok) if o],
                   edgecolors=SURFACE, linewidths=2, zorder=3)
        if key == "cost_ratio":
            lo, hi = np.nanmin(vals), np.nanmax(vals)
            pad = (hi - lo) * 0.35 + 0.01
            ax.set_xlim(lo - pad, hi + pad * 1.6)
            ax.axvline(1.0, color=TEXT_2, lw=1, ls=(0, (3, 3)), zorder=2)
            ax.hlines(y[ok], lo - pad, vals[ok], color=GRID_LINE, lw=1, zorder=1)
        else:
            ax.set_xscale("log")
            lo, hi = np.nanmin(vals), np.nanmax(vals)
            ax.set_xlim(lo / 2.5, hi * 6)
            ax.hlines(y[ok], lo / 2.5, vals[ok], color=GRID_LINE, lw=1, zorder=1)
        for yi, v, o in zip(y, vals, ok):
            if o:
                # Keep labels off the dashed 1.0 reference line.
                left = key == "cost_ratio" and v < 1.0
                ax.annotate(fmt(v), (v, yi), xytext=(-9 if left else 9, 0),
                            textcoords="offset points", va="center",
                            ha="right" if left else "left", fontsize=9, color=TEXT)
            else:
                ax.text(ax.get_xlim()[0], yi, "  no successes", va="center",
                        fontsize=9, color=TEXT_2)

    axes[0].set_yticks(y)
    axes[0].set_yticklabels(names, color=TEXT)
    fig.suptitle(title, x=0.01, ha="left", fontsize=12, fontweight="bold",
                 color=TEXT)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_convergence(histories: list[list[tuple[int, float]]],
                     ref_costs: list[float], out: Path, title: str) -> None:
    """RRT* best path cost (relative to A*) versus iteration, median and IQR."""
    iters = np.array([it for it, _ in histories[0]])
    ratios = np.array([[c / ref for _, c in h] for h, ref in zip(histories, ref_costs)])
    solved = np.isfinite(ratios)
    frac_solved = solved.mean(axis=0)
    # Only plot iterations where most maps already have a solution, otherwise
    # the median is dominated by "no path yet".
    show = frac_solved >= 0.5
    masked = np.where(solved, ratios, np.nan)
    med = np.nanmedian(masked[:, show], axis=0)
    q1 = np.nanpercentile(masked[:, show], 25, axis=0)
    q3 = np.nanpercentile(masked[:, show], 75, axis=0)
    x = iters[show]

    c = color_for("RRT*")
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    ax.fill_between(x, q1, q3, color=to_rgba(c, 0.18), lw=0)
    ax.plot(x, med, color=c, lw=2)
    ax.axhline(1.0, color=TEXT_2, lw=1, ls=(0, (3, 3)))
    ax.text(x[-1], 1.0, "A* on the 8-connected grid ", ha="right", va="bottom",
            fontsize=9, color=TEXT_2)
    ax.annotate(f"median {med[-1]:.3f}", (x[-1], med[-1]), xytext=(0, -14),
                textcoords="offset points", ha="right", fontsize=9, color=TEXT)
    ax.text(x[0], q3[0], "  middle 50% of maps", va="bottom", fontsize=9,
            color=TEXT_2)
    ax.set_xlabel("Iterations")
    ax.set_ylabel("Best path cost / A* cost")
    ax.set_title(title, loc="left")
    ax.grid(axis="y", color=GRID_LINE, lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


# ------------------------------------------------------------------ demo GIF
def animate(problem: Problem, results: list[PlanResult], out: Path,
            frames: int = 45, hold: int = 20, fps: int = 12) -> None:
    """Side-by-side animation of each planner's search on one map.

    Each panel is normalized to its own search length so they all finish
    together; the counters in the titles show the real amount of work.
    """
    import imageio.v2 as imageio

    n = len(results)
    cols = 3
    rows = math.ceil(n / cols)
    images = []
    for f in range(frames + hold):
        frac = min(1.0, (f + 1) / frames)
        fig, axes = plt.subplots(rows, cols, figsize=(3.3 * cols, 3.55 * rows))
        for ax, r in zip(np.atleast_1d(axes).ravel(), results):
            draw_map(ax, problem)
            color = color_for(r.planner)
            snaps = r.extra.get("snapshots")
            if snaps:  # RRT*: draw the actual tree at this moment, rewiring included
                k = min(len(snaps) - 1, int(frac * len(snaps)))
                parent = snaps[k]
                pts = r.extra["nodes_array"]
                segs = [(pts[parent[i]], pts[i]) for i in range(1, len(parent))]
                ax.add_collection(LineCollection(segs, colors=[to_rgba(color, 0.55)],
                                                 linewidths=0.8, zorder=3))
                count = f"{len(parent):,} nodes"
            else:
                upto = int(frac * len(r.explored))
                draw_explored(ax, r, upto)
                unit = "edges" if r.explored and isinstance(r.explored[0][0], tuple) else "expanded"
                count = f"{upto:,} {unit}"
            if frac >= 1.0:
                draw_path(ax, r.path, color, lw=2.2)
                count = f"cost {r.cost:.1f}" if r.success else "no path"
            draw_endpoints(ax, problem)
            ax.set_title(f"{r.planner}\n{count}", loc="left", fontsize=10)
        for ax in np.atleast_1d(axes).ravel()[n:]:
            ax.axis("off")
        fig.tight_layout(h_pad=1.6)
        fig.canvas.draw()
        images.append(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())
        plt.close(fig)
    imageio.mimsave(out, images, duration=1000 / fps, loop=0)
