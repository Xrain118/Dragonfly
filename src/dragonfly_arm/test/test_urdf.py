"""Physical dimensions, FK and control-contract regression checks."""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).parents[1]
ARM = [f"joint_{i}" for i in range(1, 6)]
TILT = [f"unit_{u}_{a}_joint" for u in "abc" for a in ("outer", "inner")]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return result


def fk(robot, positions=None):
    positions = positions or {}
    children = {j.find("child").get("link") for j in robot.findall("joint")}
    roots = {l.get("name") for l in robot.findall("link")} - children
    assert roots == {"world"}
    poses = {"world": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in poses:
                continue
            assert child not in poses, "Multiple parents in URDF"
            transform = origin(joint.find("origin"))
            if joint.get("type") == "revolute":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                transform[:3, :3] = transform[:3, :3] @ rotation(axis, positions.get(joint.get("name"), 0))
            poses[child] = poses[parent] @ transform
            pending.remove(joint)
            progressed = True
        assert progressed, "Disconnected/cyclic URDF"
    return poses


@pytest.fixture(scope="module")
def robot():
    result = subprocess.run(
        ["xacro", str(PACKAGE / "urdf/dragonfly_arm.urdf.xacro")],
        check=True, capture_output=True, text=True,
    )
    return ET.fromstring(result.stdout)


def test_tree_and_controller_contract(robot):
    poses = fk(robot)
    assert len(poses) == len(robot.findall("link"))
    joints = {j.get("name"): j for j in robot.findall("joint")}
    active = {name for name, j in joints.items() if j.get("type") == "revolute"}
    assert active == set(ARM + TILT)
    assert "joint_6" not in joints
    assert [joints[name].find("axis").get("xyz") for name in ARM] == [
        "0 0 1", "0 1 0", "0 1 0", "0 1 0", "0 0 1"
    ]
    controls = {j.get("name"): j for j in robot.findall("ros2_control/joint")}
    assert set(controls) == active
    config = yaml.safe_load((PACKAGE / "config/controllers.yaml").read_text())
    assert config["arm_controller"]["ros__parameters"]["joints"] == ARM
    assert config["tilt_controller"]["ros__parameters"]["joints"] == TILT
    for name in active:
        limit = joints[name].find("limit")
        params = {p.get("name"): float(p.text) for p in controls[name].findall("command_interface/param")}
        assert float(limit.get("lower")) == params["min"]
        assert float(limit.get("upper")) == params["max"]
        assert [e.get("name") for e in controls[name].findall("state_interface")] == ["position", "velocity"]


def test_owner_dimensions_and_arm_mount(robot):
    poses = fk(robot)
    centres = [poses[f"unit_{u}_mount_link"][:3, 3] for u in "abc"]
    for i in range(3):
        assert np.linalg.norm(centres[i] - centres[(i+1) % 3]) == pytest.approx(1.2)
    base = poses["uav_base_link"]
    assert base[2, 3] == pytest.approx(1.6)
    arm_in_body = np.linalg.inv(base) @ poses["base_link"]
    assert arm_in_body[:3, 3] == pytest.approx([0, 0, -0.05])
    assert arm_in_body[:3, 2] == pytest.approx([0, 0, -1], abs=1e-12)
    tip_in_arm = np.linalg.inv(poses["base_link"]) @ poses["tool0"]
    assert tip_in_arm[:3, 3] == pytest.approx([0, 0, 0.4])
    assert poses["tool0"][2, 3] > poses["left_skid"][2, 3] + 0.05


def test_five_axis_forward_kinematics(robot):
    # Independent planar prediction: three pitch joints, then axial wrist roll.
    q = [0.25, 0.15, -0.25, 0.10, 0.30]
    poses = fk(robot, dict(zip(ARM, q)))
    p = np.linalg.inv(poses["base_link"]) @ poses["tool0"]
    lengths = [0.150, 0.085, 0.035 + 0.085]
    angles = np.cumsum(q[1:4])
    reach_x = sum(length * math.sin(angle) for length, angle in zip(lengths, angles))
    reach_z = 0.045 + sum(length * math.cos(angle) for length, angle in zip(lengths, angles))
    assert p[:3, 3] == pytest.approx([reach_x*math.cos(q[0]), reach_x*math.sin(q[0]), reach_z])
    expected = rotation([0, 0, 1], q[0]) @ rotation([0, 1, 0], sum(q[1:4])) @ rotation([0, 0, 1], q[4])
    assert p[:3, :3] == pytest.approx(expected)


@pytest.mark.parametrize("outer,inner", [(0, 0), (0.3, 0), (0, -0.3), (0.3, -0.2)])
def test_tilt_order_motor_spacing_and_thrust_axes(robot, outer, inner):
    positions = {name: outer if "outer" in name else inner for name in TILT}
    poses = fk(robot, positions)
    links = {l.get("name"): l for l in robot.findall("link")}
    for unit in "abc":
        mount = poses[f"unit_{unit}_mount_link"]
        rotor_poses = [poses[f"unit_{unit}_rotor_{s}_link"] for s in ("left", "right")]
        assert np.linalg.norm(rotor_poses[0][:3, 3] - rotor_poses[1][:3, 3]) == pytest.approx(0.36)
        # R_x(outer) R_y(inner) +Z, expressed in the fixed mount frame.
        expected_z = [math.sin(inner), -math.sin(outer)*math.cos(inner), math.cos(outer)*math.cos(inner)]
        for side, pose in zip(("left", "right"), rotor_poses):
            relative = np.linalg.inv(mount) @ pose
            assert relative[:3, 2] == pytest.approx(expected_z)
            rotor = links[f"unit_{unit}_rotor_{side}_link"]
            assert len(rotor.findall("visual")) == 3
            assert float(rotor.find("collision/geometry/cylinder").get("radius")) == pytest.approx(0.116)
    if outer == 0 and inner == 0:
        all_rotors = [poses[f"unit_{u}_rotor_{s}_link"][:3, 3] for u in "abc" for s in ("left", "right")]
        for i, p in enumerate(all_rotors):
            for other in all_rotors[i+1:]:
                assert np.linalg.norm(p-other) > 2*0.116


def test_finite_positive_inertias(robot):
    for link in robot.findall("link"):
        if link.get("name") in ("world", "tool0"):
            continue
        assert link.find("visual") is not None
        assert link.find("collision") is not None
        inertial = link.find("inertial")
        assert float(inertial.find("mass").get("value")) > 0
        i = {k: float(v) for k, v in inertial.find("inertia").attrib.items()}
        matrix = np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]], [i["ixz"], i["iyz"], i["izz"]]])
        eigenvalues = np.linalg.eigvalsh(matrix)
        assert np.all(np.isfinite(eigenvalues))
        assert np.all(eigenvalues > 0)
        assert eigenvalues[-1] <= sum(eigenvalues[:-1]) + 1e-12
