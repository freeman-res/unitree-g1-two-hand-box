from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

Vector3 = Tuple[float, float, float]
Matrix4 = List[List[float]]


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: Vector3
    rpy: Vector3
    axis: Vector3
    lower: Optional[float]
    upper: Optional[float]


@dataclass(frozen=True)
class RobotModel:
    joints: Dict[str, Joint]
    links: List[str]
    children_to_joint: Dict[str, Joint]
    parent_to_joints: Dict[str, List[Joint]]


RIGHT_ARM_HINTS = ("right", "r_", "shoulder", "elbow", "wrist", "hand", "palm")


def load_urdf(path: str) -> RobotModel:
    root = ET.parse(path).getroot()
    links = [item.attrib["name"] for item in root.findall("link") if "name" in item.attrib]
    joints: Dict[str, Joint] = {}
    child_to_joint: Dict[str, Joint] = {}
    parent_to_joints: Dict[str, List[Joint]] = {}

    for item in root.findall("joint"):
        name = item.attrib.get("name", "")
        joint_type = item.attrib.get("type", "fixed")
        parent_el = item.find("parent")
        child_el = item.find("child")
        if parent_el is None or child_el is None:
            continue
        parent = parent_el.attrib["link"]
        child = child_el.attrib["link"]
        origin_el = item.find("origin")
        xyz = _parse_vec(origin_el.attrib.get("xyz", "0 0 0") if origin_el is not None else "0 0 0")
        rpy = _parse_vec(origin_el.attrib.get("rpy", "0 0 0") if origin_el is not None else "0 0 0")
        axis_el = item.find("axis")
        axis = _parse_vec(axis_el.attrib.get("xyz", "1 0 0") if axis_el is not None else "1 0 0")
        limit_el = item.find("limit")
        lower = float(limit_el.attrib["lower"]) if limit_el is not None and "lower" in limit_el.attrib else None
        upper = float(limit_el.attrib["upper"]) if limit_el is not None and "upper" in limit_el.attrib else None
        joint = Joint(name, joint_type, parent, child, xyz, rpy, axis, lower, upper)
        joints[name] = joint
        child_to_joint[child] = joint
        parent_to_joints.setdefault(parent, []).append(joint)

    return RobotModel(joints, links, child_to_joint, parent_to_joints)


def roots(model: RobotModel) -> List[str]:
    children = set(model.children_to_joint)
    return [link for link in model.links if link not in children]


def path_to_link(model: RobotModel, base: str, tip: str) -> List[Joint]:
    found: List[Joint] = []

    def visit(link: str, path: List[Joint]) -> bool:
        if link == tip:
            found.extend(path)
            return True
        for joint in model.parent_to_joints.get(link, []):
            if visit(joint.child, path + [joint]):
                return True
        return False

    if not visit(base, []):
        raise ValueError(f"No chain from {base!r} to {tip!r}")
    return found


def active_joints(chain: List[Joint]) -> List[Joint]:
    return [joint for joint in chain if joint.joint_type in ("revolute", "continuous", "prismatic")]


def fk(chain: List[Joint], values_by_name: Dict[str, float]) -> Matrix4:
    transform = identity4()
    for joint in chain:
        transform = matmul4(transform, origin_matrix(joint.xyz, joint.rpy))
        q = float(values_by_name.get(joint.name, 0.0))
        if joint.joint_type in ("revolute", "continuous"):
            transform = matmul4(transform, rotation_axis(joint.axis, q))
        elif joint.joint_type == "prismatic":
            transform = matmul4(transform, translation(tuple(v * q for v in joint.axis)))
    return transform


def position(transform: Matrix4) -> Vector3:
    return (transform[0][3], transform[1][3], transform[2][3])


def solve_ik_position(
    chain: List[Joint],
    target: Vector3,
    seed_values: Optional[List[float]] = None,
    iterations: int = 120,
    damping: float = 0.08,
    step_limit: float = 0.12,
    tolerance: float = 0.015,
) -> Tuple[List[float], float]:
    movable = active_joints(chain)
    if seed_values is None:
        q = [0.0 for _ in movable]
    else:
        if len(seed_values) != len(movable):
            raise ValueError(f"seed must have {len(movable)} values")
        q = [float(value) for value in seed_values]
    q = [_clamp_joint(movable[index], value) for index, value in enumerate(q)]

    for _ in range(iterations):
        pos = _fk_pos_for_q(chain, movable, q)
        error = [target[i] - pos[i] for i in range(3)]
        error_norm = math.sqrt(sum(v * v for v in error))
        if error_norm <= tolerance:
            return q, error_norm
        jac = _numerical_jacobian(chain, movable, q)
        dq = _damped_least_squares_step(jac, error, damping)
        norm = max(abs(value) for value in dq) if dq else 0.0
        if norm > step_limit:
            scale = step_limit / norm
            dq = [value * scale for value in dq]
        q = [_clamp_joint(movable[index], q[index] + dq[index]) for index in range(len(q))]

    pos = _fk_pos_for_q(chain, movable, q)
    error = [target[i] - pos[i] for i in range(3)]
    return q, math.sqrt(sum(v * v for v in error))


def _fk_pos_for_q(chain: List[Joint], movable: List[Joint], q: List[float]) -> Vector3:
    values = {joint.name: q[index] for index, joint in enumerate(movable)}
    return position(fk(chain, values))


def _numerical_jacobian(chain: List[Joint], movable: List[Joint], q: List[float], eps: float = 1e-4) -> List[List[float]]:
    base_pos = _fk_pos_for_q(chain, movable, q)
    jac = [[0.0 for _ in movable] for _ in range(3)]
    for index in range(len(movable)):
        q2 = list(q)
        q2[index] += eps
        pos2 = _fk_pos_for_q(chain, movable, q2)
        for axis in range(3):
            jac[axis][index] = (pos2[axis] - base_pos[axis]) / eps
    return jac


def _damped_least_squares_step(jac: List[List[float]], error: List[float], damping: float) -> List[float]:
    # dq = J^T * inv(J J^T + lambda^2 I) * error. This only inverts 3x3.
    jj_t = [[sum(jac[row][k] * jac[col][k] for k in range(len(jac[0]))) for col in range(3)] for row in range(3)]
    for i in range(3):
        jj_t[i][i] += damping * damping
    inv = inv3(jj_t)
    tmp = [sum(inv[row][col] * error[col] for col in range(3)) for row in range(3)]
    return [sum(jac[row][joint] * tmp[row] for row in range(3)) for joint in range(len(jac[0]))]


def _clamp_joint(joint: Joint, value: float) -> float:
    if joint.lower is not None:
        value = max(joint.lower, value)
    if joint.upper is not None:
        value = min(joint.upper, value)
    return value


def _parse_vec(text: str) -> Vector3:
    parts = [float(part) for part in text.replace(",", " ").split()]
    while len(parts) < 3:
        parts.append(0.0)
    return (parts[0], parts[1], parts[2])


def identity4() -> Matrix4:
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def translation(xyz: Vector3) -> Matrix4:
    mat = identity4()
    mat[0][3], mat[1][3], mat[2][3] = xyz
    return mat


def origin_matrix(xyz: Vector3, rpy: Vector3) -> Matrix4:
    return matmul4(translation(xyz), rpy_matrix(rpy))


def rpy_matrix(rpy: Vector3) -> Matrix4:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0, 0], [0, cr, -sr, 0], [0, sr, cr, 0], [0, 0, 0, 1]]
    ry = [[cp, 0, sp, 0], [0, 1, 0, 0], [-sp, 0, cp, 0], [0, 0, 0, 1]]
    rz = [[cy, -sy, 0, 0], [sy, cy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    return matmul4(matmul4(rz, ry), rx)


def rotation_axis(axis: Vector3, angle: float) -> Matrix4:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-9:
        return identity4()
    x, y, z = x / norm, y / norm, z / norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul4(a: Matrix4, b: Matrix4) -> Matrix4:
    return [[sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)] for row in range(4)]


def inv3(m: List[List[float]]) -> List[List[float]]:
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        det = 1e-12 if det >= 0 else -1e-12
    return [
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
    ]


def print_probe(model: RobotModel) -> None:
    print("ROOTS", roots(model))
    print("LINKS", len(model.links), "JOINTS", len(model.joints))
    print("RIGHT/ARM/HAND JOINTS")
    for joint in model.joints.values():
        text = " ".join((joint.name, joint.parent, joint.child)).lower()
        if any(hint in text for hint in RIGHT_ARM_HINTS):
            lo = "" if joint.lower is None else f" lower={joint.lower:.3f}"
            hi = "" if joint.upper is None else f" upper={joint.upper:.3f}"
            print(f"{joint.name}: type={joint.joint_type} parent={joint.parent} child={joint.child} axis={joint.axis}{lo}{hi}")
    print("CANDIDATE TIP LINKS")
    for link in model.links:
        low = link.lower()
        if "right" in low and any(word in low for word in ("hand", "palm", "wrist", "gripper", "finger")):
            print(link)


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe G1 URDF and do simple pure-Python FK/IK for one chain.")
    parser.add_argument("--urdf", default="config/g1_body29_hand14.urdf")
    parser.add_argument("--base-link", default="")
    parser.add_argument("--tip-link", default="")
    parser.add_argument("--joint-values", default="", help="Comma-separated active joint values in chain order.")
    parser.add_argument("--target", default="", help="x,y,z target for position IK in base-link frame.")
    parser.add_argument("--probe", action="store_true")
    args = parser.parse_args()

    model = load_urdf(args.urdf)
    if args.probe or not args.base_link or not args.tip_link:
        print_probe(model)
        return 0

    chain = path_to_link(model, args.base_link, args.tip_link)
    movable = active_joints(chain)
    print("CHAIN")
    for joint in chain:
        marker = "*" if joint in movable else " "
        print(f"{marker} {joint.name}: {joint.parent} -> {joint.child} type={joint.joint_type} axis={joint.axis} limit=({joint.lower},{joint.upper})")

    if args.joint_values:
        values = [float(part.strip()) for part in args.joint_values.split(",") if part.strip()]
    else:
        values = [0.0 for _ in movable]
    if len(values) != len(movable):
        raise ValueError(f"Need {len(movable)} joint values, got {len(values)}")
    values_by_name = {joint.name: values[index] for index, joint in enumerate(movable)}
    pose = fk(chain, values_by_name)
    print("FK_ACTIVE_JOINTS", [joint.name for joint in movable])
    print("FK_VALUES", [round(v, 5) for v in values])
    print("FK_POSITION", [round(v, 5) for v in position(pose)])

    if args.target:
        target = tuple(float(part.strip()) for part in args.target.split(",") if part.strip())
        if len(target) != 3:
            raise ValueError("--target must be x,y,z")
        q, error = solve_ik_position(chain, target, seed_values=values)
        print("IK_TARGET", [round(v, 5) for v in target])
        print("IK_VALUES", [round(v, 5) for v in q])
        print("IK_ERROR_M", round(error, 5))
        print("IK_POSITION", [round(v, 5) for v in _fk_pos_for_q(chain, movable, q)])
    return 0


if __name__ == "__main__":
    sys.exit(main())
