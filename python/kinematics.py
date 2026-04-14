import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

@dataclass
class ArmGeometry:
    base_height: float = 80.0
    shoulder_length: float = 110.0
    elbow_length: float = 110.0
    wrist_length: float = 70.0
    tool_length: float = 40.0

def deg_to_rad(value: float) -> float:
    return value * math.pi / 180.0

def _planar_to_world(radius: float, z: float, base_angle: float):
    x = radius * math.cos(base_angle)
    y = radius * math.sin(base_angle)
    return (x, y, z)

def forward_kinematics(joints_deg: Iterable[float], geometry: ArmGeometry) -> List[Tuple[float, float, float]]:
    q = list(joints_deg)
    if len(q) != 6:
        raise ValueError("Expected 6 joint angles")

    q0 = deg_to_rad(q[0] - 90.0)
    q1 = deg_to_rad(q[1] - 90.0)
    q2 = deg_to_rad(q[2] - 90.0)
    q3 = deg_to_rad(q[3] - 90.0)

    base = (0.0, 0.0, 0.0)
    shoulder = (0.0, 0.0, geometry.base_height)

    a1 = q1
    a2 = q1 + q2
    a3 = q1 + q2 + q3

    r_elbow = geometry.shoulder_length * math.cos(a1)
    z_elbow = geometry.base_height + geometry.shoulder_length * math.sin(a1)
    elbow = _planar_to_world(r_elbow, z_elbow, q0)

    r_wrist = r_elbow + geometry.elbow_length * math.cos(a2)
    z_wrist = z_elbow + geometry.elbow_length * math.sin(a2)
    wrist = _planar_to_world(r_wrist, z_wrist, q0)

    r_tool = r_wrist + (geometry.wrist_length + geometry.tool_length) * math.cos(a3)
    z_tool = z_wrist + (geometry.wrist_length + geometry.tool_length) * math.sin(a3)
    tool = _planar_to_world(r_tool, z_tool, q0)

    return [base, shoulder, elbow, wrist, tool]
