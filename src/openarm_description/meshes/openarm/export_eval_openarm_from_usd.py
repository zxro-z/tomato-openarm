#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


SOURCE_USD = Path("/home/zxro/arena/lerobot/src/lerobot/assets/openarm_use/openarm_half_tesollo_tactile.usd")
XACRO_PATH = Path("/home/zxro/tomato/src/openarm_description/urdf/openarm_eval_bimanual_right_tesollo.urdf.xacro")
OUTPUT_ROOT = Path("/home/zxro/tomato/src/openarm_description/meshes/eval_usd")
REPORT_PATH = OUTPUT_ROOT / "openarm_eval_report.json"

USD_EXTRA_PYTHONPATH = (
    "/home/zxro/isaacsim510/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311",
    "/home/zxro/isaacsim510/extscache/omni.usd.schema.physx-107.3.26+107.3.3.lx64.r.cp311.u353",
)


@dataclass
class JointRecord:
    name: str
    joint_type: str
    parent: str
    child: str
    axis: list[float] | None
    origin_xyz: list[float]
    origin_rpy: list[float]
    lower: float | None
    upper: float | None


def _round_list(values: list[float], digits: int = 9) -> list[float]:
    return [round(float(v), digits) for v in values]


def _quat_to_rpy(quat: Gf.Quatf | Gf.Quatd) -> list[float]:
    q = quat.GetNormalized()
    w = float(q.GetReal())
    x, y, z = (float(v) for v in q.GetImaginary())
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return _round_list([roll, pitch, yaw])


def _axis_vector(axis_name: str | None) -> list[float] | None:
    if axis_name is None:
        return None
    axis_name = axis_name.upper()
    if axis_name == "X":
        return [1.0, 0.0, 0.0]
    if axis_name == "Y":
        return [0.0, 1.0, 0.0]
    if axis_name == "Z":
        return [0.0, 0.0, 1.0]
    return None


def _fan_triangulate(face_indices: list[int]) -> list[tuple[int, int, int]]:
    if len(face_indices) < 3:
        return []
    return [(face_indices[0], face_indices[i], face_indices[i + 1]) for i in range(1, len(face_indices) - 1)]


def _transform_point(matrix: Gf.Matrix4d, point: Gf.Vec3f | Gf.Vec3d) -> tuple[float, float, float]:
    transformed = matrix.Transform(Gf.Vec3d(point[0], point[1], point[2]))
    return (float(transformed[0]), float(transformed[1]), float(transformed[2]))


def extract_usd_joints(stage: Usd.Stage) -> dict[str, JointRecord]:
    joints: dict[str, JointRecord] = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        name = prim.GetName()
        parent_targets = joint.GetBody0Rel().GetTargets()
        child_targets = joint.GetBody1Rel().GetTargets()
        if not parent_targets or not child_targets:
            continue
        joint_type = prim.GetTypeName().replace("Physics", "").replace("Joint", "").lower()
        axis = None
        lower = None
        upper = None
        if prim.IsA(UsdPhysics.RevoluteJoint):
            revolute = UsdPhysics.RevoluteJoint(prim)
            axis = _axis_vector(revolute.GetAxisAttr().Get())
            lower = math.radians(float(revolute.GetLowerLimitAttr().Get()))
            upper = math.radians(float(revolute.GetUpperLimitAttr().Get()))
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            prismatic = UsdPhysics.PrismaticJoint(prim)
            axis = _axis_vector(prismatic.GetAxisAttr().Get())
            lower = float(prismatic.GetLowerLimitAttr().Get())
            upper = float(prismatic.GetUpperLimitAttr().Get())
        joints[name] = JointRecord(
            name=name,
            joint_type=joint_type,
            parent=parent_targets[0].name,
            child=child_targets[0].name,
            axis=axis,
            origin_xyz=_round_list(list(joint.GetLocalPos0Attr().Get())),
            origin_rpy=_quat_to_rpy(joint.GetLocalRot0Attr().Get()),
            lower=round(lower, 9) if lower is not None else None,
            upper=round(upper, 9) if upper is not None else None,
        )
    return joints


def extract_usd_hierarchy(stage: Usd.Stage) -> dict[str, object]:
    default_prim = stage.GetDefaultPrim()
    robot_root = stage.GetPrimAtPath("/openarm/openarm_body_link0")
    right_mount = stage.GetPrimAtPath("/openarm/tesollo_right/rl_dg_mount")
    return {
        "default_prim": default_prim.GetPath().pathString,
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "up_axis": UsdGeom.GetStageUpAxis(stage),
        "robot_root_prim": robot_root.GetPath().pathString if robot_root.IsValid() else None,
        "tesollo_mount_prim": right_mount.GetPath().pathString if right_mount.IsValid() else None,
    }


def extract_visual_meshes(stage: Usd.Stage, output_dir: Path) -> tuple[dict[str, list[str]], list[str]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    visuals_by_link: dict[str, list[str]] = {}
    unresolved_visual_prims: list[str] = []

    for prim in stage.Traverse():
        if prim.GetName() != "visuals":
            continue
        parent = prim.GetParent()
        if not parent.IsValid():
            continue
        link_name = parent.GetName()
        meshes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh)]
        if not meshes:
            unresolved_visual_prims.append(prim.GetPath().pathString)
            continue

        parent_world = cache.GetLocalToWorldTransform(parent)
        world_to_parent = parent_world.GetInverse()
        vertices: list[tuple[float, float, float]] = []
        faces: list[tuple[int, int, int]] = []
        vertex_offset = 0

        for mesh_prim in meshes:
            mesh = UsdGeom.Mesh(mesh_prim)
            points = mesh.GetPointsAttr().Get() or []
            face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get() or []
            face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get() or []
            if not points:
                continue
            mesh_to_parent = cache.GetLocalToWorldTransform(mesh_prim) * world_to_parent
            transformed = []
            for point in points:
                x, y, z = _transform_point(mesh_to_parent, point)
                transformed.append((x * meters_per_unit, y * meters_per_unit, z * meters_per_unit))
            vertices.extend(transformed)

            cursor = 0
            for count in face_vertex_counts:
                local_face = [face_vertex_indices[cursor + i] + 1 + vertex_offset for i in range(count)]
                cursor += count
                faces.extend(_fan_triangulate(local_face))
            vertex_offset += len(transformed)

        if not vertices or not faces:
            unresolved_visual_prims.append(prim.GetPath().pathString)
            continue

        output_path = output_dir / f"{link_name}_visual.obj"
        with output_path.open("w", encoding="utf-8") as f:
            f.write(f"# source={SOURCE_USD}\n")
            f.write(f"# link={link_name}\n")
            f.write("o visual\n")
            for x, y, z in vertices:
                f.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
            for i, j, k in faces:
                f.write(f"f {i} {j} {k}\n")
        visuals_by_link[link_name] = [str(output_path)]

    return visuals_by_link, sorted(set(unresolved_visual_prims))


def parse_xacro_joints(xacro_path: Path) -> dict[str, JointRecord]:
    tree = ET.parse(xacro_path)
    root = tree.getroot()
    arg_defaults: dict[str, str] = {}
    for arg in root.findall("{http://www.ros.org/wiki/xacro}arg"):
        if "name" in arg.attrib and "default" in arg.attrib:
            arg_defaults[arg.attrib["name"]] = arg.attrib["default"]

    def _resolve_scalar_list(raw: str) -> list[float]:
        if raw.startswith("$(arg ") and raw.endswith(")"):
            arg_name = raw[6:-1].strip()
            raw = arg_defaults[arg_name]
        return [float(v) for v in raw.split()]

    joints: dict[str, JointRecord] = {}
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        joint_type = joint.attrib["type"]
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if "xyz" in origin.attrib:
                xyz = _resolve_scalar_list(origin.attrib["xyz"])
            if "rpy" in origin.attrib:
                rpy = _resolve_scalar_list(origin.attrib["rpy"])
        axis = None
        axis_node = joint.find("axis")
        if axis_node is not None and "xyz" in axis_node.attrib:
            axis = [float(v) for v in axis_node.attrib["xyz"].split()]
        lower = None
        upper = None
        limit = joint.find("limit")
        if limit is not None:
            if "lower" in limit.attrib:
                lower = float(limit.attrib["lower"])
            if "upper" in limit.attrib:
                upper = float(limit.attrib["upper"])
        joints[name] = JointRecord(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            axis=_round_list(axis) if axis is not None else None,
            origin_xyz=_round_list(xyz),
            origin_rpy=_round_list(rpy),
            lower=round(lower, 9) if lower is not None else None,
            upper=round(upper, 9) if upper is not None else None,
        )
    return joints


def compare_joint_records(current: dict[str, JointRecord], usd: dict[str, JointRecord]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in sorted(set(current) | set(usd)):
        current_joint = current.get(name)
        usd_joint = usd.get(name)
        if current_joint is None or usd_joint is None:
            rows.append({"joint": name, "status": "MISSING", "current": asdict(current_joint) if current_joint else None, "eval": asdict(usd_joint) if usd_joint else None})
            continue
        row = {"joint": name, "status": "PASS", "fields": {}}
        for field in ("joint_type", "parent", "child", "axis", "origin_xyz", "origin_rpy", "lower", "upper"):
            current_value = getattr(current_joint, field)
            usd_value = getattr(usd_joint, field)
            match = current_value == usd_value
            row["fields"][field] = {"current": current_value, "eval": usd_value, "match": match}
            if not match:
                row["status"] = "FAIL"
        rows.append(row)
    return rows


def extract_eval_initial_pose(joints: dict[str, JointRecord]) -> dict[str, dict[str, dict[str, float]]]:
    tree = ET.parse(XACRO_PATH)
    _ = tree  # keep local parse aligned with report generation path
    initial_deg = {
        "openarm_left_joint1": 20.0,
        "openarm_left_joint2": 0.0,
        "openarm_left_joint3": 0.0,
        "openarm_left_joint4": 60.0,
        "openarm_left_joint5": 0.0,
        "openarm_left_joint6": 0.0,
        "openarm_left_joint7": -44.0,
        "openarm_right_joint1": -20.0,
        "openarm_right_joint2": 40.0,
        "openarm_right_joint3": -10.0,
        "openarm_right_joint4": 60.0,
        "openarm_right_joint5": 30.0,
        "openarm_right_joint6": 15.0,
        "openarm_right_joint7": 44.0,
    }
    grouped = {"LEFT": {}, "RIGHT": {}}
    for name, deg in initial_deg.items():
        side = "LEFT" if "_left_" in name else "RIGHT"
        grouped[side][name] = {"deg": deg, "rad": round(math.radians(deg), 9)}
    return grouped


def main() -> None:
    stage = Usd.Stage.Open(str(SOURCE_USD), load=Usd.Stage.LoadAll)
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {SOURCE_USD}")

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    usd_joints = extract_usd_joints(stage)
    current_joints = parse_xacro_joints(XACRO_PATH)
    hierarchy = extract_usd_hierarchy(stage)
    visuals_by_link, unresolved_visual_prims = extract_visual_meshes(stage, OUTPUT_ROOT / "visuals")
    report = {
        "source_usd": str(SOURCE_USD),
        "xacro_path": str(XACRO_PATH),
        "hierarchy": hierarchy,
        "usd_joint_count": len(usd_joints),
        "current_joint_count": len(current_joints),
        "joint_comparison": compare_joint_records(current_joints, usd_joints),
        "visual_mesh_exports": visuals_by_link,
        "unresolved_visual_prims": unresolved_visual_prims,
        "eval_initial_pose": extract_eval_initial_pose(usd_joints),
    }
    REPORT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"report": str(REPORT_PATH), "visual_dir": str(OUTPUT_ROOT / "visuals")}, indent=2))


if __name__ == "__main__":
    main()
