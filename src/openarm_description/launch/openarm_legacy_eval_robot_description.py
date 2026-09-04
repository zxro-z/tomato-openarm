#!/usr/bin/env python3
"""Expand the canonical OpenArm xacro without its environment table link.

The table belongs to openarm_baseline's legacy-eval scene publisher.  Keeping
it out of robot_description gives each scene element one visual/TF owner while
leaving the canonical robot geometry and kinematics untouched.
"""

from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as ET


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} CANONICAL_URDF_XACRO", file=sys.stderr)
        return 2

    expanded = subprocess.run(["xacro", sys.argv[1]], check=True, text=True, capture_output=True).stdout
    root = ET.fromstring(expanded)
    excluded = {"table_visual_link", "table_visual_joint"}
    for element in list(root):
        if element.tag in {"link", "joint"} and element.get("name") in excluded:
            root.remove(element)
    sys.stdout.write(ET.tostring(root, encoding="unicode"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
