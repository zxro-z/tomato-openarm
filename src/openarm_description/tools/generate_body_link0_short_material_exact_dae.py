#!/usr/bin/env python3
"""Shorten the original body DAE while preserving each material partition.

The original Collada geometry groups are the source of truth.  Only the dark
central extrusion is shortened; the two upper assemblies are rigidly lowered.
No STL geometry or nearest-surface material classification is used.
"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": COLLADA_NS}
PROFILE_GEOMETRY = "geometry_bb7832a8_28b3_4689_b3c7_a13b7f895c32-mesh"
UPPER_GEOMETRIES = {
    "geometry_78cbfdda_a447_4b48_8deb_f5e944fb69d5-mesh",
    "geometry_6c0caaf0_b8c6_49e3_838b_a85397b99d04-mesh",
}
PROFILE_OLD_TOP_MM = 758.0
HEIGHT_DELTA_MM = 193.0


def main() -> int:
    package_root = Path(__file__).resolve().parents[1]
    source = package_root / "meshes/body/v10/visual/body_link0.dae"
    output = package_root / "meshes/body/v10/visual/body_link0_short_material_exact.dae"

    ET.register_namespace("", COLLADA_NS)
    tree = ET.parse(source)
    root = tree.getroot()
    for geometry in root.findall(".//c:library_geometries/c:geometry", NS):
        geometry_id = geometry.attrib["id"]
        source_element = geometry.find(f"c:mesh/c:source[@id='{geometry_id}-positions']", NS)
        float_array = source_element.find("c:float_array", NS)
        positions = np.fromstring(float_array.text, sep=" ", dtype=np.float64).reshape(-1, 3)

        if geometry_id == PROFILE_GEOMETRY:
            top = np.isclose(positions[:, 2], PROFILE_OLD_TOP_MM, atol=1e-8)
            if not np.any(top) or not np.all(np.isclose(positions[:, 2], 8.0, atol=1e-8) | top):
                raise RuntimeError("central profile no longer has the expected two-level extrusion topology")
            positions[top, 2] -= HEIGHT_DELTA_MM
            transform = "profile top z=758 mm -> 565 mm"
        elif geometry_id in UPPER_GEOMETRIES:
            positions[:, 2] -= HEIGHT_DELTA_MM
            transform = "rigid z=-193 mm"
        else:
            transform = "unchanged"

        float_array.text = " ".join(f"{value:.17g}" for row in positions for value in row)
        print(f"{geometry_id}: {transform}")

    tree.write(output, encoding="utf-8", xml_declaration=True)
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
