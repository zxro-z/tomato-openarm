#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from pxr import Gf, Usd, UsdGeom


SOURCE_USD = Path("/home/zxro/arena/lerobot/src/lerobot/assets/openarm_use/table.usd")
OUTPUT_OBJ = Path("/home/zxro/tomato/src/openarm_description/meshes/table/table_visual.obj")
ROOT_PRIM_PATH = "/table"


@dataclass
class MeshSummary:
    path: str
    point_count: int
    face_count: int


def _transform_point(matrix: Gf.Matrix4d, point: Gf.Vec3f) -> tuple[float, float, float]:
    transformed = matrix.Transform(Gf.Vec3d(point[0], point[1], point[2]))
    return (float(transformed[0]), float(transformed[1]), float(transformed[2]))


def _fan_triangulate(face_indices: list[int]) -> list[tuple[int, int, int]]:
    if len(face_indices) < 3:
        return []
    return [(face_indices[0], face_indices[i], face_indices[i + 1]) for i in range(1, len(face_indices) - 1)]


def export_table_obj(source_usd: Path = SOURCE_USD, output_obj: Path = OUTPUT_OBJ) -> None:
    stage = Usd.Stage.Open(str(source_usd), load=Usd.Stage.LoadAll)
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {source_usd}")

    root = stage.GetPrimAtPath(ROOT_PRIM_PATH)
    if not root:
        raise RuntimeError(f"Missing root prim: {ROOT_PRIM_PATH}")

    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    if meters_per_unit <= 0.0:
        raise RuntimeError(f"Invalid metersPerUnit: {meters_per_unit}")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_world = cache.GetLocalToWorldTransform(root)
    world_to_root = root_world.GetInverse()

    all_vertices: list[tuple[float, float, float]] = []
    all_faces: list[tuple[int, int, int]] = []
    summaries: list[MeshSummary] = []
    vertex_offset = 0

    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue

        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get() or []
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get() or []
        face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get() or []
        local_to_root = cache.GetLocalToWorldTransform(prim) * world_to_root

        transformed_points = []
        for point in points:
            x, y, z = _transform_point(local_to_root, point)
            transformed_points.append((x * meters_per_unit, y * meters_per_unit, z * meters_per_unit))

        all_vertices.extend(transformed_points)

        cursor = 0
        mesh_faces = 0
        for count in face_vertex_counts:
            local_face = [face_vertex_indices[cursor + i] + 1 + vertex_offset for i in range(count)]
            cursor += count
            for tri in _fan_triangulate(local_face):
                all_faces.append(tri)
                mesh_faces += 1

        summaries.append(MeshSummary(prim.GetPath().pathString, len(points), mesh_faces))
        vertex_offset += len(transformed_points)

    if not all_vertices or not all_faces:
        raise RuntimeError("No mesh geometry exported from USD")

    output_obj.parent.mkdir(parents=True, exist_ok=True)
    with output_obj.open("w", encoding="utf-8") as f:
        f.write("# Exported from LeRobot table.usd\n")
        f.write(f"# source={source_usd}\n")
        f.write(f"# root_prim={ROOT_PRIM_PATH}\n")
        f.write(f"# meters_per_unit={meters_per_unit}\n")
        f.write("o table_visual\n")
        for x, y, z in all_vertices:
            f.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
        for i, j, k in all_faces:
            f.write(f"f {i} {j} {k}\n")

    mins = [min(v[i] for v in all_vertices) for i in range(3)]
    maxs = [max(v[i] for v in all_vertices) for i in range(3)]
    size = [maxs[i] - mins[i] for i in range(3)]

    print(f"default prim: {stage.GetDefaultPrim().GetPath()}")
    print(f"metersPerUnit: {meters_per_unit}")
    print(f"upAxis: {UsdGeom.GetStageUpAxis(stage)}")
    print(f"mesh prim count: {len(summaries)}")
    for summary in summaries:
        print(f"mesh: {summary.path} points={summary.point_count} triangles={summary.face_count}")
    print(f"obj path: {output_obj}")
    print(f"obj bounds min: {mins}")
    print(f"obj bounds max: {maxs}")
    print(f"obj bounds size: {size}")


if __name__ == "__main__":
    export_table_obj()
