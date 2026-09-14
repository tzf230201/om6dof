#!/usr/bin/env python3
"""Generate validated top-down setup cards for the four-object pilot."""

from __future__ import annotations

import argparse
import csv
import math
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Circle, Ellipse, FancyArrowPatch, FancyBboxPatch, Rectangle


OBJECT_STYLE = {
    "bottle": {"face": "#4C78A8", "marker": "bottle"},
    "cup": {"face": "#F28E2B", "marker": "cup"},
    "banana": {"face": "#EDC948", "marker": "banana"},
    "remote": {"face": "#B279A2", "marker": "remote"},
}


@dataclass(frozen=True)
class SceneCard:
    run_id: str
    scene_id: str
    reset_index: int
    rows_by_order: tuple[dict[str, str], ...]


def read_manifest(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"manifest is empty: {path}")
    return rows


def read_experiment(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"experiment YAML is not a mapping: {path}")
    return data


def position_vectors(experiment: dict) -> dict[str, tuple[float, float, float]]:
    offsets = experiment.get("positions", {}).get("suggested_offsets_m", {})
    vectors: dict[str, tuple[float, float, float]] = {}
    for position, vector in offsets.items():
        if not isinstance(vector, list) or len(vector) != 3:
            raise ValueError(f"position {position} must have exactly three offsets")
        vectors[str(position)] = tuple(float(value) for value in vector)
    return vectors


def validate_sources(
    manifest_rows: list[dict[str, str]], experiment: dict
) -> tuple[list[SceneCard], dict[str, tuple[float, float, float]], float]:
    experiment_config = experiment.get("experiment", {})
    objects = experiment.get("objects", [])
    object_by_id = {str(item["id"]): item for item in objects}
    if len(object_by_id) != len(objects):
        raise ValueError("experiment.yaml contains duplicate object IDs")

    positions = position_vectors(experiment)
    if set(positions) != {"P1", "P2", "P3", "P4"}:
        raise ValueError(f"expected P1-P4 in suggested offsets, got {sorted(positions)}")
    if experiment.get("positions", {}).get("calibration_required") is not True:
        raise ValueError("scene cards require positions.calibration_required: true")
    spacing_m = float(experiment_config["minimum_object_spacing_m"])
    if spacing_m <= 0:
        raise ValueError("minimum_object_spacing_m must be positive")
    for first_index, first in enumerate(sorted(positions)):
        for second in sorted(positions)[first_index + 1 :]:
            distance = math.dist(positions[first], positions[second])
            if distance + 1e-12 < spacing_m:
                raise ValueError(
                    f"suggested offsets violate minimum spacing: {first}-{second} "
                    f"is {distance * 1000:.1f} mm, required {spacing_m * 1000:.1f} mm"
                )

    groups: OrderedDict[str, list[dict[str, str]]] = OrderedDict()
    class_ids_by_object: dict[str, set[str]] = {}
    for row_number, row in enumerate(manifest_rows, start=2):
        missing = [
            field
            for field in (
                "opportunity_id",
                "run_id",
                "reset_index",
                "scene_id",
                "target_order",
                "target_object",
                "target_instance_id",
                "coco_class",
                "coco_class_id",
                "object_position",
            )
            if not row.get(field)
        ]
        if missing:
            raise ValueError(f"manifest row {row_number} is missing {missing}")
        object_id = row["target_instance_id"]
        if object_id not in object_by_id:
            raise ValueError(f"manifest row {row_number} has unknown object ID {object_id}")
        expected_object = object_by_id[object_id]
        if row["target_object"] != str(expected_object["name"]):
            raise ValueError(
                f"manifest row {row_number}: target_object {row['target_object']} "
                f"does not match {object_id} name {expected_object['name']}"
            )
        if row["coco_class"] != str(expected_object["coco_class"]):
            raise ValueError(
                f"manifest row {row_number}: COCO class {row['coco_class']} "
                f"does not match {object_id} class {expected_object['coco_class']}"
            )
        if row["object_position"] not in positions:
            raise ValueError(
                f"manifest row {row_number} has unknown position {row['object_position']}"
            )
        class_ids_by_object.setdefault(object_id, set()).add(row["coco_class_id"])
        groups.setdefault(row["run_id"], []).append(row)

    for object_id, class_ids in class_ids_by_object.items():
        if len(class_ids) != 1:
            raise ValueError(f"{object_id} has inconsistent COCO class IDs: {sorted(class_ids)}")

    expected_resets = int(experiment_config["scene_resets"])
    expected_targets = int(experiment_config["target_opportunities_per_reset"])
    if len(groups) != expected_resets:
        raise ValueError(f"expected {expected_resets} runs, manifest has {len(groups)}")
    if expected_targets != len(object_by_id) or expected_targets != len(positions):
        raise ValueError("targets per reset must match both object and position counts")

    expected_classes = {
        value.strip() for value in str(experiment_config["target_classes"]).split(",")
    }
    yaml_classes = {str(item["coco_class"]) for item in objects}
    if expected_classes != yaml_classes:
        raise ValueError("experiment target_classes does not match the object definitions")

    cards: list[SceneCard] = []
    seen_scenes: set[str] = set()
    for expected_reset, (run_id, group) in enumerate(groups.items(), start=1):
        if len(group) != expected_targets:
            raise ValueError(f"{run_id} has {len(group)} rows, expected {expected_targets}")
        reset_indices = {int(row["reset_index"]) for row in group}
        scene_ids = {row["scene_id"] for row in group}
        if reset_indices != {expected_reset}:
            raise ValueError(
                f"{run_id} reset index is {sorted(reset_indices)}, expected {expected_reset}"
            )
        if len(scene_ids) != 1:
            raise ValueError(f"{run_id} has multiple scene IDs: {sorted(scene_ids)}")
        scene_id = next(iter(scene_ids))
        if scene_id in seen_scenes:
            raise ValueError(f"scene ID is reused: {scene_id}")
        seen_scenes.add(scene_id)

        ordered = sorted(group, key=lambda row: int(row["target_order"]))
        orders = [int(row["target_order"]) for row in ordered]
        if orders != list(range(1, expected_targets + 1)):
            raise ValueError(f"{run_id} target order is invalid: {orders}")
        if {row["target_instance_id"] for row in ordered} != set(object_by_id):
            raise ValueError(f"{run_id} does not contain every configured object exactly once")
        if {row["object_position"] for row in ordered} != set(positions):
            raise ValueError(f"{run_id} does not fill P1-P4 exactly once")
        for row in ordered:
            expected_opportunity = f"{run_id}_t{int(row['target_order']):02d}"
            if row["opportunity_id"] != expected_opportunity:
                raise ValueError(
                    f"{run_id} opportunity {row['opportunity_id']} should be {expected_opportunity}"
                )
        cards.append(SceneCard(run_id, scene_id, expected_reset, tuple(ordered)))
    return cards, positions, spacing_m


def draw_object(axis, x: float, y: float, object_class: str, face: str) -> None:
    edge = "#1F2933"
    if object_class == "bottle":
        axis.add_patch(
            FancyBboxPatch(
                (x - 0.016, y - 0.027),
                0.032,
                0.052,
                boxstyle="round,pad=0.002,rounding_size=0.007",
                facecolor=face,
                edgecolor=edge,
                linewidth=1.3,
                zorder=5,
            )
        )
        axis.add_patch(
            Rectangle(
                (x - 0.007, y + 0.025),
                0.014,
                0.008,
                facecolor=face,
                edgecolor=edge,
                linewidth=1.2,
                zorder=5,
            )
        )
    elif object_class == "cup":
        axis.add_patch(Circle((x, y), 0.022, facecolor=face, edgecolor=edge, linewidth=1.3, zorder=5))
        axis.add_patch(Circle((x + 0.026, y), 0.010, facecolor="none", edgecolor=edge, linewidth=1.3, zorder=4))
    elif object_class == "banana":
        axis.add_patch(
            Ellipse(
                (x, y),
                width=0.054,
                height=0.022,
                angle=25,
                facecolor=face,
                edgecolor=edge,
                linewidth=1.3,
                zorder=5,
            )
        )
    else:
        axis.add_patch(
            Rectangle(
                (x - 0.031, y - 0.013),
                0.062,
                0.026,
                angle=0,
                facecolor=face,
                edgecolor=edge,
                linewidth=1.3,
                zorder=5,
            )
        )


def add_dimension(
    axis,
    start: tuple[float, float],
    end: tuple[float, float],
    label: str,
    label_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="<->",
        mutation_scale=10,
        linewidth=1.2,
        color="#334E68",
        zorder=3,
    )
    axis.add_patch(arrow)
    midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
    axis.text(
        midpoint[0] + label_offset[0],
        midpoint[1] + label_offset[1],
        label,
        ha="center",
        va="center",
        fontsize=8.5,
        color="#243B53",
        bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.5, "alpha": 0.9},
        zorder=6,
    )


def render_card(
    card: SceneCard,
    positions: dict[str, tuple[float, float, float]],
    spacing_m: float,
    experiment: dict,
) -> plt.Figure:
    experiment_config = experiment["experiment"]
    reset_count = int(experiment_config["scene_resets"])
    fig = plt.figure(figsize=(11.69, 8.27), facecolor="#F7F9FC")
    plan = fig.add_axes([0.045, 0.175, 0.625, 0.70])
    info = fig.add_axes([0.695, 0.175, 0.275, 0.70])
    footer = fig.add_axes([0.045, 0.035, 0.925, 0.105])

    fig.text(
        0.045,
        0.935,
        f"TOP-DOWN SCENE CARD  |  {card.run_id}  |  {card.scene_id}",
        fontsize=18,
        fontweight="bold",
        color="#102A43",
    )
    fig.text(
        0.955,
        0.937,
        f"RESET {card.reset_index:02d}/{reset_count:02d}",
        fontsize=11,
        fontweight="bold",
        color="#486581",
        ha="right",
    )
    fig.text(
        0.045,
        0.902,
        "Placement and target-order reference - not a measured trajectory",
        fontsize=10,
        color="#627D98",
    )

    plan.set_xlim(-0.16, 0.16)
    plan.set_ylim(-0.18, 0.16)
    plan.set_aspect("equal")
    plan.axis("off")
    plan.add_patch(
        FancyBboxPatch(
            (-0.145, -0.125),
            0.29,
            0.27,
            boxstyle="round,pad=0.003,rounding_size=0.006",
            facecolor="#FFFDF7",
            edgecolor="#829AB1",
            linewidth=1.5,
            zorder=0,
        )
    )
    plan.plot([-0.009, 0.009], [0, 0], color="#526D82", linewidth=1.0, zorder=2)
    plan.plot([0, 0], [-0.009, 0.009], color="#526D82", linewidth=1.0, zorder=2)
    plan.text(0.005, -0.013, "table_center_world", fontsize=7.5, color="#526D82")
    plan.annotate("+x", xy=(0.055, 0), xytext=(0.018, 0), arrowprops={"arrowstyle": "->", "color": "#526D82"}, fontsize=8, color="#526D82")
    plan.annotate("+y", xy=(0, 0.055), xytext=(0, 0.016), arrowprops={"arrowstyle": "->", "color": "#526D82"}, fontsize=8, color="#526D82")

    position_to_row = {row["object_position"]: row for row in card.rows_by_order}
    for position_id in ("P1", "P2", "P3", "P4"):
        row = position_to_row[position_id]
        x, y, _ = positions[position_id]
        object_class = row["coco_class"]
        style = OBJECT_STYLE.get(object_class, {"face": "#9FB3C8", "marker": "generic"})
        draw_object(plan, x, y, object_class, style["face"])
        order = int(row["target_order"])
        badge_x = x - 0.027 if x < 0 else x + 0.027
        badge_y = y + 0.029
        plan.add_patch(Circle((badge_x, badge_y), 0.012, facecolor="#D64545", edgecolor="white", linewidth=1.2, zorder=7))
        plan.text(badge_x, badge_y, str(order), ha="center", va="center", fontsize=8.5, fontweight="bold", color="white", zorder=8)
        label_y = y + 0.054 if y >= 0 else y - 0.054
        vertical_alignment = "bottom" if y >= 0 else "top"
        plan.text(
            x,
            label_y,
            f"{position_id}  {row['target_instance_id']}\n{row['coco_class']} [COCO {row['coco_class_id']}]\norder {order}",
            ha="center",
            va=vertical_alignment,
            fontsize=8.4,
            color="#102A43",
            fontweight="bold",
            linespacing=1.25,
            zorder=8,
        )

    horizontal_distance_mm = math.dist(positions["P1"], positions["P2"]) * 1000
    vertical_distance_mm = math.dist(positions["P2"], positions["P3"]) * 1000
    add_dimension(
        plan,
        (positions["P1"][0], 0.116),
        (positions["P2"][0], 0.116),
        f"suggested {horizontal_distance_mm:.0f} mm",
        label_offset=(0, 0.011),
    )
    add_dimension(
        plan,
        (0.126, positions["P2"][1]),
        (0.126, positions["P3"][1]),
        f"suggested {vertical_distance_mm:.0f} mm",
        label_offset=(0.0, 0.0),
    )
    plan.text(
        -0.135,
        0.025,
        f"MIN. SPACING >= {spacing_m * 1000:.0f} mm",
        fontsize=8.2,
        color="#334E68",
        fontweight="bold",
    )

    plan.annotate(
        "",
        xy=(0, -0.128),
        xytext=(0, -0.164),
        arrowprops={"arrowstyle": "-|>", "linewidth": 2.2, "color": "#007C91"},
    )
    plan.text(
        0,
        -0.174,
        "ROBOT + CAMERA SIDE",
        ha="center",
        va="top",
        fontsize=9.5,
        fontweight="bold",
        color="#007C91",
    )
    plan.text(
        0,
        -0.188,
        "orientation reference - confirm during table calibration",
        ha="center",
        va="top",
        fontsize=7.5,
        color="#526D82",
        clip_on=False,
    )

    info.set_xlim(0, 1)
    info.set_ylim(0, 1)
    info.axis("off")
    info.add_patch(
        FancyBboxPatch(
            (0.01, 0.52),
            0.98,
            0.47,
            boxstyle="round,pad=0.012,rounding_size=0.02",
            facecolor="#FFFFFF",
            edgecolor="#9FB3C8",
            linewidth=1.2,
        )
    )
    info.text(0.06, 0.945, "TARGET ORDER", fontsize=11, fontweight="bold", color="#102A43")
    y = 0.885
    for row in card.rows_by_order:
        style = OBJECT_STYLE.get(row["coco_class"], {"face": "#9FB3C8"})
        info.add_patch(Circle((0.085, y + 0.005), 0.025, facecolor=style["face"], edgecolor="#243B53", linewidth=0.8))
        info.text(0.085, y + 0.005, row["target_order"], ha="center", va="center", fontsize=8, fontweight="bold", color="white")
        info.text(
            0.14,
            y + 0.005,
            f"{row['target_instance_id']}  {row['coco_class']}  [{row['object_position']}]",
            va="center",
            fontsize=9.5,
            fontweight="bold",
            color="#243B53",
        )
        info.text(0.14, y - 0.030, row["opportunity_id"], va="center", fontsize=7.4, color="#627D98")
        y -= 0.092

    info.add_patch(
        FancyBboxPatch(
            (0.01, 0.02),
            0.98,
            0.45,
            boxstyle="round,pad=0.012,rounding_size=0.02",
            facecolor="#EEF7F8",
            edgecolor="#007C91",
            linewidth=1.5,
        )
    )
    info.text(0.06, 0.425, "SYNCHRONIZATION SLATE", fontsize=11, fontweight="bold", color="#006778")
    slate_lines = [
        f"RUN ID       {card.run_id}",
        f"SCENE ID     {card.scene_id}",
        f"RESET        {card.reset_index:02d}/{reset_count:02d}",
        "DATE (UTC)   __________________",
        "START FLASH  __________________",
        "END FLASH    __________________",
        "OUTSIDE CAM  __________________",
        "ROBOT POV    __________________",
    ]
    y = 0.37
    for line in slate_lines:
        info.text(0.07, y, line, fontsize=8.7, family="monospace", color="#243B53")
        y -= 0.043
    info.text(
        0.07,
        0.045,
        "Hold slate in both views before recording.",
        fontsize=7.7,
        color="#486581",
        fontweight="bold",
    )

    footer.set_xlim(0, 1)
    footer.set_ylim(0, 1)
    footer.axis("off")
    footer.add_patch(
        FancyBboxPatch(
            (0.0, 0.04),
            1.0,
            0.90,
            boxstyle="round,pad=0.01,rounding_size=0.015",
            facecolor="#FFF4D6",
            edgecolor="#D9A441",
            linewidth=1.3,
        )
    )
    offsets = " | ".join(
        f"{position}: ({vector[0] * 1000:+.0f}, {vector[1] * 1000:+.0f}, {vector[2] * 1000:+.0f}) mm"
        for position, vector in sorted(positions.items())
    )
    footer.text(
        0.018,
        0.67,
        "CALIBRATION REQUIRED: dimensions below are suggested offsets from table_center_world, not surveyed table coordinates.",
        fontsize=9.1,
        fontweight="bold",
        color="#7C4A03",
    )
    footer.text(0.018, 0.33, offsets, fontsize=8.2, family="monospace", color="#5C3B0A")
    footer.text(
        0.982,
        0.08,
        f"Protocol v{experiment_config['protocol_version']} | "
        f"{str(experiment_config['mode']).replace('_', ' ')} | physical execution "
        f"{'enabled' if experiment_config['physical_execution_enabled'] else 'disabled'}",
        ha="right",
        fontsize=7.4,
        color="#7C4A03",
    )
    return fig


def generate(
    root: Path,
    output_directory: Path,
    pdf_path: Path,
    dpi: int,
) -> tuple[list[Path], Path]:
    manifest_path = root / "trial_manifest.csv"
    experiment_path = root / "experiment.yaml"
    rows = read_manifest(manifest_path)
    experiment = read_experiment(experiment_path)
    cards, positions, spacing_m = validate_sources(rows, experiment)

    output_directory.mkdir(parents=True, exist_ok=True)
    pdf_path.parent.mkdir(parents=True, exist_ok=True)
    expected_pngs = [output_directory / f"scene_card_{card.run_id}.png" for card in cards]
    unexpected = sorted(set(output_directory.glob("scene_card_*.png")) - set(expected_pngs))
    if unexpected:
        raise ValueError(
            "refusing to leave stale scene cards beside this manifest: "
            + ", ".join(path.name for path in unexpected)
        )

    with PdfPages(
        pdf_path,
        metadata={
            "Title": "Four-object experiment top-down scene cards",
            "Subject": "Validated placement, target order, orientation, and synchronization slates",
            "Author": "OM6DOF experiment tooling",
        },
    ) as pdf:
        for card, png_path in zip(cards, expected_pngs):
            fig = render_card(card, positions, spacing_m, experiment)
            fig.savefig(png_path, dpi=dpi, facecolor=fig.get_facecolor())
            pdf.savefig(fig, facecolor=fig.get_facecolor())
            plt.close(fig)

    if len(expected_pngs) != 12 or any(not path.is_file() or path.stat().st_size == 0 for path in expected_pngs):
        raise RuntimeError("expected exactly 12 non-empty PNG scene cards")
    if not pdf_path.is_file() or pdf_path.stat().st_size == 0:
        raise RuntimeError("multipage PDF was not created")
    print(f"Validated {len(cards)} scene cards against {len(rows)} manifest rows.")
    print(pdf_path)
    for path in expected_pngs:
        print(path)
    return expected_pngs, pdf_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="four_object_pick directory",
    )
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument("--pdf", type=Path)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    if args.dpi <= 0:
        parser.error("--dpi must be positive")

    rows = read_manifest(args.root / "trial_manifest.csv")
    experiment = read_experiment(args.root / "experiment.yaml")
    cards, _, _ = validate_sources(rows, experiment)
    if args.validate_only:
        print(f"Validated {len(cards)} scene cards against {len(rows)} manifest rows.")
        return

    output_directory = args.output_directory or args.root / "figures" / "scene_cards"
    pdf_path = args.pdf or args.root / "figures" / "scene_cards.pdf"
    generate(args.root, output_directory, pdf_path, args.dpi)


if __name__ == "__main__":
    main()
