#!/usr/bin/env python3
"""Create a material-preserving DAE for the verified shortened body mesh.

The target triangles are copied exactly from body_link0_short.stl, preserving
the previously validated body silhouette and bounds.  Their material classes
are recovered from the closest original body_link0.dae surface.  The original
Collada effect, material, and instance-material binding libraries are kept.
"""

from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree


COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": COLLADA_NS}


def qname(name: str) -> str:
    return f"{{{COLLADA_NS}}}{name}"


def load_binary_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path} is not a binary STL with a valid facet count")

    faces = np.empty((count, 3, 3), dtype=np.float64)
    normals = np.empty((count, 3), dtype=np.float64)
    for index, offset in enumerate(range(84, 84 + 50 * count, 50)):
        normals[index] = struct.unpack_from("<3f", data, offset)
        faces[index] = np.array(struct.unpack_from("<9f", data, offset + 12)).reshape(3, 3)
    return faces, normals


def original_triangle_centroids(root: ET.Element):
    centroids = []
    labels = []
    geometry_ids = []
    material_symbols = []
    geometries = root.findall(".//c:library_geometries/c:geometry", NS)
    for label, geometry in enumerate(geometries):
        mesh = geometry.find("c:mesh", NS)
        geometry_id = geometry.attrib["id"]
        positions = np.fromstring(
            mesh.find(f"c:source[@id='{geometry_id}-positions']/c:float_array", NS).text,
            sep=" ",
            dtype=np.float64,
        ).reshape(-1, 3)
        triangles = mesh.find("c:triangles", NS)
        indices = np.fromstring(triangles.find("c:p", NS).text, sep=" ", dtype=np.int64).reshape(-1, 6)[:, ::2]
        centroids.append(positions[indices].mean(axis=1))
        labels.append(np.full(len(indices), label, dtype=np.int32))
        geometry_ids.append(geometry_id)
        material_symbols.append(triangles.attrib["material"])
    return np.concatenate(centroids), np.concatenate(labels), geometry_ids, material_symbols, geometries


def add_source(mesh: ET.Element, source_id: str, values: list[tuple[float, float, float]]):
    source = ET.SubElement(mesh, qname("source"), {"id": source_id})
    array_id = f"{source_id}-array"
    float_array = ET.SubElement(source, qname("float_array"), {"id": array_id, "count": str(3 * len(values))})
    # Preserve the binary-STL float coordinates exactly through text encoding.
    float_array.text = " ".join(f"{value:.17g}" for row in values for value in row)
    common = ET.SubElement(source, qname("technique_common"))
    accessor = ET.SubElement(common, qname("accessor"), {"source": f"#{array_id}", "count": str(len(values)), "stride": "3"})
    for axis in ("X", "Y", "Z"):
        ET.SubElement(accessor, qname("param"), {"name": axis, "type": "float"})


def replace_geometry_mesh(geometry: ET.Element, face_indices: np.ndarray, faces: np.ndarray, normals: np.ndarray, material: str):
    geometry_id = geometry.attrib["id"]
    old_mesh = geometry.find(qname("mesh"))
    geometry.remove(old_mesh)
    mesh = ET.SubElement(geometry, qname("mesh"))

    positions: list[tuple[float, float, float]] = []
    face_normals: list[tuple[float, float, float]] = []
    position_ids: dict[tuple[float, float, float], int] = {}
    normal_ids: dict[tuple[float, float, float], int] = {}
    indices: list[int] = []

    for face_index in face_indices:
        normal = tuple(float(value) for value in normals[face_index])
        norm = float(np.linalg.norm(normal))
        if norm > 0.0:
            normal = tuple(value / norm for value in normal)
        normal_id = normal_ids.setdefault(normal, len(face_normals))
        if normal_id == len(face_normals):
            face_normals.append(normal)
        for vertex in faces[face_index]:
            point = tuple(float(value) for value in vertex)
            point_id = position_ids.setdefault(point, len(positions))
            if point_id == len(positions):
                positions.append(point)
            indices.extend((point_id, normal_id))

    position_source = f"{geometry_id}-positions"
    normal_source = f"{geometry_id}-normals"
    vertices_id = f"{geometry_id}-vertices"
    add_source(mesh, position_source, positions)
    add_source(mesh, normal_source, face_normals)
    vertices = ET.SubElement(mesh, qname("vertices"), {"id": vertices_id})
    ET.SubElement(vertices, qname("input"), {"semantic": "POSITION", "source": f"#{position_source}"})
    triangles = ET.SubElement(mesh, qname("triangles"), {"material": material, "count": str(len(face_indices))})
    ET.SubElement(triangles, qname("input"), {"semantic": "VERTEX", "source": f"#{vertices_id}", "offset": "0"})
    ET.SubElement(triangles, qname("input"), {"semantic": "NORMAL", "source": f"#{normal_source}", "offset": "1"})
    ET.SubElement(triangles, qname("p")).text = " ".join(map(str, indices))


def main() -> int:
    package_root = Path(__file__).resolve().parents[1]
    original_path = package_root / "meshes/body/v10/visual/body_link0.dae"
    short_stl_path = package_root / "meshes/body/v10/visual/body_link0_short.stl"
    output_path = package_root / "meshes/body/v10/visual/body_link0_short.dae"

    ET.register_namespace("", COLLADA_NS)
    tree = ET.parse(original_path)
    root = tree.getroot()
    original_centroids, original_labels, geometry_ids, material_symbols, geometries = original_triangle_centroids(root)
    faces, normals = load_binary_stl(short_stl_path)

    distances, nearest = cKDTree(original_centroids).query(faces.mean(axis=1), workers=-1)
    labels = original_labels[nearest]

    # A fully removed original component can leave an empty <triangles> block,
    # which some Collada importers reject.  Keep every original geometry node
    # valid by moving one face from an equivalent material class.  This has no
    # visual effect because both source and target use the same material.
    for label, material in enumerate(material_symbols):
        if np.any(labels == label):
            continue
        donor = next(index for index, candidate in enumerate(material_symbols) if candidate == material and np.any(labels == index))
        labels[np.flatnonzero(labels == donor)[0]] = label

    for label, geometry in enumerate(geometries):
        replace_geometry_mesh(geometry, np.flatnonzero(labels == label), faces, normals, material_symbols[label])

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    bounds_min = faces.reshape(-1, 3).min(axis=0)
    bounds_max = faces.reshape(-1, 3).max(axis=0)
    print(f"wrote {output_path}")
    print(f"triangles={len(faces)} aabb_mm_min={bounds_min.tolist()} aabb_mm_max={bounds_max.tolist()}")
    print(
        "nearest_original_surface_distance_mm="
        f"p50={np.quantile(distances, 0.50):.6g} p95={np.quantile(distances, 0.95):.6g} max={distances.max():.6g}"
    )
    for label, geometry_id in enumerate(geometry_ids):
        print(f"{geometry_id} material={material_symbols[label]} triangles={int((labels == label).sum())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
