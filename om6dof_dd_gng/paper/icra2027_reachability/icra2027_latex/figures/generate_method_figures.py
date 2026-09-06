"""Generate reproducible method figures for the ICRA reachability paper."""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, FancyArrowPatch, FancyBboxPatch, Rectangle


OUT = Path(__file__).resolve().parent

BLUE = "#2F77B4"
GREEN = "#2A8B62"
ORANGE = "#B87900"
MAGENTA = "#A13B72"
RED = "#C74440"
INK = "#17324D"
GRAY = "#A5B2BF"
LIGHT = "#EDF3F7"


# ----------------------------------------------------------------------------
# Faithful 2-D ports of the deployed samplers.
#
# These mirror om6dof_dd_gng/include/om6dof_dd_gng/reachability_graph.hpp so the
# figure shows what the paper actually runs. Earlier revisions of this script
# illustrated "GNG" with k-means over an artificially density-weighted pool;
# both were misleading, because the deployed pipeline trains real Growing
# Neural Gas on validity-filtered Halton samples, which are near-uniform.
# ----------------------------------------------------------------------------

MASK64 = (1 << 64) - 1


def splitmix64(value: int) -> int:
    value = (value + 0x9E3779B97F4A7C15) & MASK64
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & MASK64
    return value ^ (value >> 31)


def halton_digit_permutation(base: int, stream_seed: int, dimension: int) -> list[int]:
    """Generalized Halton digit permutation (zero stays fixed)."""
    permutation = list(range(base))
    if stream_seed == 0:
        return permutation
    state = splitmix64(stream_seed ^ splitmix64(dimension + 1) ^ base)
    for i in range(base - 1, 1, -1):
        state = splitmix64(state)
        j = 1 + state % i
        permutation[i], permutation[j] = permutation[j], permutation[i]
    return permutation


def halton_radical_inverse(index: int, base: int, permutation: list[int]) -> float:
    result = 0.0
    fraction = 1.0 / base
    while index > 0:
        result += fraction * permutation[index % base]
        index //= base
        fraction /= base
    return result


def halton(n: int, bases: tuple[int, int] = (2, 3), offset: int = 1,
           stream_seed: int = 0) -> np.ndarray:
    """Digit-permuted Halton stream, indexed as the C++ sampler indexes it."""
    permutations = [halton_digit_permutation(base, stream_seed, dimension)
                    for dimension, base in enumerate(bases)]
    return np.array(
        [[halton_radical_inverse(offset + i + 1, base, permutations[d])
          for d, base in enumerate(bases)]
         for i in range(n)],
        dtype=float,
    )


def valid_region(points: np.ndarray) -> np.ndarray:
    x, y = points[:, 0], points[:, 1]
    outer = ((x - 0.50) / 0.47) ** 2 + ((y - 0.50) / 0.43) ** 2 <= 1.0
    notch = (x > 0.43) & (x < 0.58) & (y < 0.42)
    return outer & ~notch


def valid_halton_stream(count: int, offset: int = 19, stream_seed: int = 0) -> np.ndarray:
    """The accepted-candidate stream: Halton samples that pass the predicate."""
    raw = halton(count * 4, offset=offset, stream_seed=stream_seed)
    return raw[valid_region(raw)][:count]


def growing_neural_gas(samples: np.ndarray, max_units: int, *, max_epochs: int = 4,
                       insertion_interval: int = 20, max_edge_age: int = 50,
                       winner_learning_rate: float = 0.05,
                       neighbor_learning_rate: float = 0.0006,
                       error_reduction: float = 0.5,
                       error_decay: float = 0.995) -> np.ndarray:
    """Deterministic GNG, line-for-line equivalent to growingNeuralGas() in
    reachability_graph.hpp. Defaults are the deployed config/topo_gng.yaml
    values, so the panels inherit the paper's actual quantizer dynamics."""
    second_seed = int(np.argmax(((samples - samples[0]) ** 2).sum(axis=1)))
    units = [samples[0].copy(), samples[second_seed].copy()]
    errors = [0.0, 0.0]
    ages = [[-1, 0], [0, -1]]
    update_count = 0

    for _ in range(max_epochs):
        for sample in samples:
            distances = [float(((sample - unit) ** 2).sum()) for unit in units]
            order = sorted(range(len(units)), key=lambda i: (distances[i], i))
            winner, runner_up = order[0], order[1]

            errors[winner] += distances[winner]
            for neighbor in range(len(units)):
                if ages[winner][neighbor] >= 0:
                    ages[winner][neighbor] += 1
                    ages[neighbor][winner] = ages[winner][neighbor]
            units[winner] += winner_learning_rate * (sample - units[winner])
            for neighbor in range(len(units)):
                if neighbor != winner and ages[winner][neighbor] >= 0:
                    units[neighbor] += neighbor_learning_rate * (sample - units[neighbor])
            ages[winner][runner_up] = 0
            ages[runner_up][winner] = 0
            for neighbor in range(len(units)):
                if ages[winner][neighbor] > max_edge_age:
                    ages[winner][neighbor] = -1
                    ages[neighbor][winner] = -1

            update_count += 1
            if len(units) < max_units and update_count % insertion_interval == 0:
                high_error = int(np.argmax(errors))
                partner = -1
                for neighbor in range(len(units)):
                    if ages[high_error][neighbor] >= 0 and (
                            partner < 0 or errors[neighbor] > errors[partner]):
                        partner = neighbor
                if partner < 0:
                    best = -1.0
                    for candidate in range(len(units)):
                        if candidate == high_error:
                            continue
                        gap = float(((units[high_error] - units[candidate]) ** 2).sum())
                        if gap > best:
                            best, partner = gap, candidate
                inserted = 0.5 * (units[high_error] + units[partner])
                for row in ages:
                    row.append(-1)
                ages.append([-1] * (len(units) + 1))
                new_index = len(units)
                units.append(inserted)
                errors[high_error] *= error_reduction
                errors[partner] *= error_reduction
                errors.append(errors[high_error])
                ages[high_error][partner] = -1
                ages[partner][high_error] = -1
                ages[high_error][new_index] = 0
                ages[new_index][high_error] = 0
                ages[partner][new_index] = 0
                ages[new_index][partner] = 0

            errors = [error * error_decay for error in errors]

    return np.array(units)


def stratified_guard_indices(sample_count: int, requested_count: int) -> list[int]:
    """Mid-stratum guard selection, mirroring stratifiedGuardIndices()."""
    count = min(sample_count, requested_count)
    return [min(sample_count - 1, int((2 * i + 1) * sample_count // (2 * count)))
            for i in range(count)]


# The goal test is a workspace one (end-effector within r_I of the target), so
# in configuration space its preimage is several disjoint sets -- one per
# inverse-kinematic branch. The paper turns on exactly this: branches are never
# merged just because their end-effector positions coincide. Drawing a single
# blob here would contradict that, so both branches of one target are shown.
IK_BRANCHES = ((0.82, 0.72, 0.095, 0.075), (0.30, 0.75, 0.080, 0.062))


def draw_configuration_panel(ax, title: str, subtitle: str, nodes: np.ndarray,
                             node_color: str, pool: np.ndarray,
                             guards: np.ndarray | None = None,
                             rejected: np.ndarray | None = None) -> None:
    ax.scatter(pool[:, 0], pool[:, 1], s=2.0, color="#D7E0E7", alpha=0.55,
               linewidths=0, rasterized=True,
               label="accepted candidate stream")
    ax.add_patch(Rectangle((0.43, 0.0), 0.15, 0.42, facecolor=RED,
                           edgecolor="none", alpha=0.14))
    ax.text(0.505, 0.18, "invalid\nregion", ha="center", va="center",
            fontsize=7, color=RED, weight="bold")
    for index, (cx, cy, rx, ry) in enumerate(IK_BRANCHES):
        ax.add_patch(Ellipse((cx, cy), 2 * rx, 2 * ry, facecolor=MAGENTA,
                             edgecolor=MAGENTA, alpha=0.10, linewidth=1.2))
        ax.scatter([cx], [cy], s=58, marker="*", color=MAGENTA,
                   edgecolor="white", linewidth=0.5, zorder=7)
        ax.text(cx, cy + ry + 0.02, f"IK branch {'AB'[index]}", ha="center",
                va="bottom", fontsize=6.6, color=MAGENTA)
    ax.text(0.56, 0.955, "one workspace target, two branches", ha="center",
            va="top", fontsize=6.6, color=MAGENTA, style="italic")
    ax.scatter(nodes[:, 0], nodes[:, 1], s=18, color=node_color,
               edgecolor="white", linewidth=0.35, zorder=4,
               label="retained roadmap node")
    if rejected is not None and len(rejected):
        ax.scatter(rejected[:, 0], rejected[:, 1], s=22, marker="x",
                   color=RED, linewidth=1.0, zorder=6,
                   label="prototype rejected as invalid")
    if guards is not None:
        ax.scatter(guards[:, 0], guards[:, 1], s=16, marker="s", color=ORANGE,
                   edgecolor="white", linewidth=0.3, zorder=5,
                   label="deterministic guard")
    ax.scatter([0.18], [0.24], s=58, marker="*", color=GREEN,
               edgecolor="white", linewidth=0.5, zorder=7)
    ax.text(0.18, 0.185, "start anchor", ha="center", va="top",
            fontsize=6.6, color=GREEN)
    ax.set_title(title, fontsize=10.2, color=INK, weight="bold", pad=5)
    ax.text(0.5, -0.10, subtitle, transform=ax.transAxes, ha="center",
            va="top", fontsize=7.6, color="#3E5263")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlabel("normalized joint projection $q_a$", fontsize=7.5, labelpad=2)
    ax.set_ylabel("$q_b$", fontsize=7.5, labelpad=1)
    for spine in ax.spines.values():
        spine.set_color("#B8C5CF")
        spine.set_linewidth(0.7)


def roadmap_construction() -> None:
    # Marker budgets are the deployed 800-node composition divided by ~11, which
    # keeps the prototype:guard ratio (199:599 -> 18:54) and the samples-per-unit
    # ratio (4000/798 -> 350/70) of the real runs.
    node_budget, guard_budget = 72, 54
    prototype_budget = node_budget - guard_budget

    # The accepted candidate stream: validity-filtered Halton, i.e. near-uniform
    # over the valid set. There is no dense cluster for GNG to chase.
    training = valid_halton_stream(350, offset=19)

    gng_units = growing_neural_gas(training, node_budget)
    gng_keep = valid_region(gng_units)
    gng_nodes, gng_rejected = gng_units[gng_keep], gng_units[~gng_keep]

    guarded_units = growing_neural_gas(training, prototype_budget)
    guarded_keep = valid_region(guarded_units)
    guarded_nodes = guarded_units[guarded_keep]
    guarded_rejected = guarded_units[~guarded_keep]
    guards = training[stratified_guard_indices(len(training), guard_budget)]

    # Halton/PRM fills its budget directly from a digit-permuted stream.
    direct = valid_halton_stream(node_budget, offset=121, stream_seed=137)

    fig, axes = plt.subplots(1, 3, figsize=(12.4, 3.95), constrained_layout=False)
    fig.patch.set_facecolor("white")
    draw_configuration_panel(
        axes[0], "GNG-quantized PRM",
        "2 anchors + 798 prototypes; learned GNG adjacency is discarded",
        gng_nodes, BLUE, training, rejected=gng_rejected,
    )
    draw_configuration_panel(
        axes[1], "Guarded GNG",
        "2 anchors + 199 prototypes + 599 low-discrepancy guards",
        guarded_nodes, BLUE, training, guards=guards, rejected=guarded_rejected,
    )
    draw_configuration_panel(
        axes[2], "Halton/PRM",
        "2 anchors + 798 direct digit-permuted Halton nodes",
        direct, GRAY, training,
    )
    fig.subplots_adjust(left=0.045, right=0.985, top=0.775, bottom=0.255, wspace=0.20)
    fig.suptitle("Method-specific node construction under the same 800-node budget",
                 x=0.5, y=0.985, fontsize=13.2, color=INK, weight="bold")
    fig.text(0.5, 0.888,
             "Two-dimensional analogue run with the deployed deterministic GNG and Halton code; marker counts are scaled, labels give the 800-node composition.",
             ha="center", va="center", fontsize=8.4, color="#536979")
    # Merge across panels: the rejected-prototype marker only occurs in the pure
    # GNG panel and the guard marker only in the guarded one, so a single
    # panel's handles would leave one of them unexplained.
    merged: dict[str, object] = {}
    for axis in axes:
        for handle, label in zip(*axis.get_legend_handles_labels()):
            merged.setdefault(label, handle)
    fig.legend(list(merged.values()), list(merged.keys()), loc="lower center",
               ncol=len(merged), bbox_to_anchor=(0.5, 0.093), frameon=False,
               fontsize=7.8, handletextpad=0.4, columnspacing=1.6)
    fig.text(0.5, 0.035,
             "All three node sets use the same validated $k$-nearest-neighbor connection rule, edge limits, and collision predicates.",
             ha="center", va="center", fontsize=8.6, color=INK, weight="bold")
    for ext in ("pdf", "png"):
        fig.savefig(OUT / f"roadmap_construction.{ext}", dpi=320,
                    bbox_inches="tight", facecolor="white")
    plt.close(fig)


def rounded_box(ax, xy, wh, title, body, edge, face, title_color=INK,
                body_size=6.9, inset=0.012):
    """Draw a labelled panel.

    ``inset``, ``title``, and ``body`` offsets are expressed in the figure-wide
    axes of ``query_pipeline`` (a full-canvas [0, 1] axes), so the vertical
    spacing below is calibrated against that canvas rather than the default
    subplot box. Keeping the geometry explicit is what stops the fourth body
    line from spilling through the bottom border.
    """
    x, y = xy
    w, h = wh
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.010,rounding_size=0.016",
        linewidth=1.25, edgecolor=edge, facecolor=face,
    )
    ax.add_patch(patch)
    ax.text(x + inset, y + h - 0.035, title, ha="left", va="top",
            fontsize=8.6, color=title_color, weight="bold")
    ax.text(x + inset, y + h - 0.078, body, ha="left", va="top",
            fontsize=body_size, color="#253B4D", linespacing=1.30)
    return patch


def arrow(ax, start, end, color=INK, rad=0.0, label=None, label_xy=None):
    p = FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=11,
                        linewidth=1.25, color=color,
                        connectionstyle=f"arc3,rad={rad}")
    ax.add_patch(p)
    if label:
        lx, ly = label_xy if label_xy is not None else (
            (start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        ax.text(lx, ly, label, fontsize=7.1, color=color, ha="center",
                va="center", weight="bold",
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white",
                          edgecolor="none", alpha=0.92))


def query_pipeline() -> None:
    # A full-canvas axes keeps one axes unit equal to one figure fraction, so
    # every offset below is a directly predictable fraction of the 12.4 x 4.6
    # canvas. The default subplot margins used to shrink the drawing area to
    # roughly 77% of that, which is what pushed body text through the panel
    # borders and slid the safety banner underneath panels 6a/6b.
    fig = plt.figure(figsize=(12.4, 4.6))
    fig.patch.set_facecolor("white")
    ax = fig.add_axes((0.0, 0.0, 1.0, 1.0))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.text(0.019, 0.972, "Environment-conditioned graph query and lazy exact validation",
            fontsize=13.0, color=INK, weight="bold", va="top")
    ax.text(0.019, 0.921,
            "A query preserves the 6-DoF configuration branch; failed exact edges are disabled only for the current environment revision.",
            fontsize=8.3, color="#536979", va="top")

    stage_w, stage_gap, stage_x0 = 0.162, 0.038, 0.019
    stage_y, stage_h = 0.615, 0.225
    xs = [stage_x0 + index * (stage_w + stage_gap) for index in range(5)]

    rounded_box(ax, (xs[0], stage_y), (stage_w, stage_h), "1  Atomic query",
                "$q_s$ measured start\nsemantic target $t$\nobstacle points/segments\nscene + graph revision",
                BLUE, "#EAF3FA")
    rounded_box(ax, (xs[1], stage_y), (stage_w, stage_h), "2  Candidate goals",
                r"EE distance $\leq r_I$" "\ncollision-free node\nsame start component\nretain every IK branch",
                MAGENTA, "#F8EDF3")
    rounded_box(ax, (xs[2], stage_y), (stage_w, stage_h), "3  Broad phase",
                "cached body capsules\npoint/segment distance\nblock nodes and edges\nno roadmap rebuild",
                ORANGE, "#FCF4E5")
    rounded_box(ax, (xs[3], stage_y), (stage_w, stage_h), "4  Graph search",
                "Dijkstra over enabled edges\njoint-space accumulated cost\nselect reachable target node\nreconstruct route",
                GREEN, "#EAF5EF")
    rounded_box(ax, (xs[4], stage_y), (stage_w, stage_h), "5  Exact validation",
                "interpolate every route edge\nMoveIt RobotState + FCL\nself + environment collision\nmaximum 20 replans",
                MAGENTA, "#F8EDF3")

    stage_mid = stage_y + stage_h / 2.0
    for index in range(4):
        arrow(ax, (xs[index] + stage_w + 0.012, stage_mid),
              (xs[index + 1] - 0.012, stage_mid), color=INK)

    outcome_y, outcome_h = 0.245, 0.195
    outcome_top = outcome_y + outcome_h
    rounded_box(ax, (0.600, outcome_y), (0.205, outcome_h), "6a  Edge rejected",
                "blacklist failed edge\nfor this environment revision\nthen rerun Dijkstra",
                RED, "#FBEDEC")
    rounded_box(ax, (0.835, outcome_y), (0.146, outcome_h), "6b  Accepted",
                "exact-valid path\nprivate preview topic\nno controller endpoint",
                GREEN, "#EAF5EF")

    arrow(ax, (0.872, stage_y - 0.010), (0.762, outcome_top + 0.010), color=RED,
          label="collision", label_xy=(0.830, 0.532))
    arrow(ax, (0.641, outcome_top + 0.010), (0.662, stage_y - 0.010), color=RED,
          label="replan", label_xy=(0.611, 0.532))
    arrow(ax, (0.930, stage_y - 0.010), (0.925, outcome_top + 0.010), color=GREEN,
          label="all states valid", label_xy=(0.948, 0.532))

    ax.text(0.019, 0.412, "Validity hierarchy", fontsize=8.5, color=INK,
            weight="bold", va="top")
    ax.text(0.019, 0.356,
            r"capsules (fast, conservative)  $\rightarrow$  graph route  $\rightarrow$  interpolated mesh collision (authoritative)",
            fontsize=8.0, color="#334B5D", va="top")

    # Full-width banner on its own row: the outcome panels end at y=0.245, so
    # centring it here keeps it clear of them instead of running behind 6a/6b.
    ax.text(0.5, 0.105,
            "Safety boundary: benchmark output is a trajectory preview only; it does not publish to /arm_controller and has no action client.",
            fontsize=7.8, color=RED, weight="bold", ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.40", facecolor="#FBEDEC",
                      edgecolor=RED, linewidth=0.9))

    for ext in ("pdf", "png"):
        fig.savefig(OUT / f"lazy_query_pipeline.{ext}", dpi=320,
                    bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    roadmap_construction()
    query_pipeline()
    print("Generated roadmap_construction and lazy_query_pipeline (PDF + PNG).")
