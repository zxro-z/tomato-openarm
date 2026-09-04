#!/usr/bin/env python3
"""Generate the OpenArm-only URDF used by the RViz smoke test.

The source xacro remains the canonical evaluated OpenArm description.  It also
contains optional end-effectors and a table, so this helper filters the expanded
URDF down to its body and two seven-axis arm chains before it is passed to
robot_state_publisher.
"""

from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as ET


def keep_names() -> tuple[set[str], set[str]]:
    links = {"world", "openarm_body_link0"}
    joints = {"openarm_body_world_joint"}
    for side in ("left", "right"):
        links.update(f"openarm_{side}_link{index}" for index in range(8))
        joints.add(f"openarm_{side}_openarm_body_link0_joint")
        joints.update(f"openarm_{side}_joint{index}" for index in range(1, 8))
    return links, joints


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} CANONICAL_URDF_XACRO", file=sys.stderr)
        return 2

    expanded = subprocess.run(
        ["xacro", sys.argv[1]], check=True, text=True, capture_output=True
    ).stdout
    root = ET.fromstring(expanded)
    allowed_links, allowed_joints = keep_names()

    for element in list(root):
        if element.tag == "link" and element.get("name") not in allowed_links:
            root.remove(element)
        elif element.tag == "joint" and element.get("name") not in allowed_joints:
            root.remove(element)

    sys.stdout.write(ET.tostring(root, encoding="unicode"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
