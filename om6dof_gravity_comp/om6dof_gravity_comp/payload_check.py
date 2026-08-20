#!/usr/bin/env python3
"""Gravity torque with the wrist payload disabled versus enabled.

Read-only. Renders the URDF, builds the same model twice -- once as written,
once with the payload's inertial stripped -- and prints what the payload adds
at several postures. Nothing is commanded to any motor.

The point of the comparison is to answer one question with numbers: does the
missing camera explain the missing torque, or does it not? If the payload adds
far less than the shortfall you see on the real arm, the remaining gap is in
the torque constant or in friction, and raising the current limit would only
be papering over whichever of those it is.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import subprocess

import numpy as np
from ament_index_python.packages import get_package_share_directory

from .gravity_model import GravityModel

# Postures chosen so the payload's lever arm about joint 2 varies from nothing
# to nearly everything; a payload that only ever showed up in one of these
# would be a sign the frame handling was wrong.
POSTURES = {
    "all zero (arm straight up)": [0, 0, 0, 0, 0, 0],
    "READY": [0, -39, 78, 0, 51, 0],
    "reaching forward (J2 -90)": [0, -90, 0, 0, 0, 0],
    "forward, elbow out": [0, -90, 45, 0, 45, 0],
    "arm horizontal, wrist down": [0, -90, 0, 0, 90, 0],
}


def render_urdf(package: str, rel: str) -> str:
    path = os.path.join(get_package_share_directory(package), rel)
    result = subprocess.run(["xacro", path], capture_output=True, text=True,
                            check=False)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr.strip()}")
    return result.stdout


def strip_payload(urdf_str: str) -> str:
    """The same robot with the payload weightless, for an honest A/B.

    Only the inertial block goes: the link and joint stay, so the kinematics
    are identical between the two models and any difference in torque can only
    come from the mass.
    """
    def blank(match: re.Match) -> str:
        return re.sub(r"\s*<inertial>.*?</inertial>", "", match.group(0),
                      flags=re.S)

    return re.sub(r'<link name="d405_payload_link">.*?</link>', blank,
                  urdf_str, flags=re.S)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf-package", default="om6dof_description")
    parser.add_argument("--urdf-file", default="urdf/om6dof.urdf.xacro")
    args = parser.parse_args()

    urdf = render_urdf(args.urdf_package, args.urdf_file)
    if "d405_payload_link" not in urdf:
        raise SystemExit("no d405_payload_link in the URDF; nothing to compare")

    with_payload = GravityModel(urdf)
    without = GravityModel(strip_payload(urdf))
    added = with_payload.total_mass() - without.total_mass()

    print(f"arm mass in model      : {without.total_mass():.4f} kg")
    print(f"payload declared       : {added:.4f} kg")
    if added <= 0.0:
        print("\nPayload mass is still 0.0 -- nothing has been measured yet.")
        print("Fill in om6dof_description/config/payload.yaml, rebuild")
        print("om6dof_description, and run this again.")
        return

    for name, degrees in POSTURES.items():
        q = np.radians(degrees)
        a = without.torques(q)
        b = with_payload.torques(q)
        print(f"\n{name}   q = {degrees} deg")
        print(f"  {'joint':7s}{'without':>10s}{'with':>10s}"
              f"{'added Nm':>11s}{'added raw':>11s}")
        for i in range(6):
            # Raw ticks shown only to size the effect against the limit you
            # already know; the torque constant here is the node's initial
            # approximation, not a calibrated value.
            kt = 0.61 if i < 4 else 0.40
            raw = (b[i] - a[i]) / kt * 1000.0 / 2.69
            print(f"  joint{i + 1:<2d}{a[i]:10.4f}{b[i]:10.4f}"
                  f"{b[i] - a[i]:11.4f}{raw:11.1f}")


if __name__ == "__main__":
    main()
