#!/usr/bin/env python3
"""Generate the fixed six-panel experiment documentation layout."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch


PANELS = [
    ("A", "Outside camera", "robot + table + operator\nphysical outcome and safety context", "#dceefa"),
    ("B", "Robot POV", "registered RGB-D view\nframe/time overlay", "#e7f4e4"),
    ("C", "YOLO semantics", "boxes + class + confidence\n2 Hz inference", "#fff0d7"),
    ("D", "Environment topology", "world DD-GNG nodes/edges\nsemantic node colours", "#f5e3ef"),
    ("E", "Reachability + path", "reachable nodes + selected goal\nexact-valid preview", "#e6e4f8"),
    ("F", "Performance", "stage latency + CPU/GPU/RAM\ntrial result and event timeline", "#f0f1f3"),
]


def main() -> None:
    output_dir = Path(__file__).resolve().parent
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.5))
    fig.patch.set_facecolor("white")
    for axis, (letter, title, detail, colour) in zip(axes.flat, PANELS):
        axis.set_xlim(0, 1)
        axis.set_ylim(0, 1)
        axis.axis("off")
        panel = FancyBboxPatch(
            (0.03, 0.05), 0.94, 0.90,
            boxstyle="round,pad=0.012,rounding_size=0.035",
            linewidth=1.8, edgecolor="#183b5b", facecolor=colour,
        )
        axis.add_patch(panel)
        axis.text(0.09, 0.84, letter, fontsize=19, fontweight="bold", color="#087ea4")
        axis.text(0.20, 0.84, title, fontsize=14, fontweight="bold", color="#17324d")
        axis.text(0.50, 0.45, detail, ha="center", va="center", fontsize=11, color="#34495e")
        axis.text(0.50, 0.14, "UTC + monotonic timestamp", ha="center", fontsize=8.5, color="#5d6d7e")

    fig.suptitle(
        "Synchronized visual documentation — four-object semantic-topology experiment",
        fontsize=16, fontweight="bold", color="#17324d", y=0.98,
    )
    fig.text(
        0.5, 0.02,
        "Keep run ID, target object, stage, and frame timestamp visible in every recorded panel.",
        ha="center", fontsize=10, color="#34495e",
    )
    plt.tight_layout(rect=(0.02, 0.05, 0.98, 0.94), h_pad=1.0, w_pad=0.7)
    fig.savefig(output_dir / "visual_capture_layout.png", dpi=220, bbox_inches="tight")
    fig.savefig(output_dir / "visual_capture_layout.pdf", bbox_inches="tight")
    print(output_dir / "visual_capture_layout.png")


if __name__ == "__main__":
    main()
