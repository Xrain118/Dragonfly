"""Structural checks for the generated Dragonfly arm URDF."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


XACRO_FILE = Path(__file__).parents[1] / "urdf" / "dragonfly_arm.urdf.xacro"
EXPECTED_JOINTS = [f"joint_{index}" for index in range(1, 7)]
EXPECTED_LINKS = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "tool0",
]


def generated_robot() -> ET.Element:
    completed = subprocess.run(
        ["xacro", str(XACRO_FILE), "use_sim_time:=true"],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(completed.stdout)


def test_six_axis_chain_and_inertials() -> None:
    robot = generated_robot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    links = {link.attrib["name"]: link for link in robot.findall("link")}

    assert set(EXPECTED_JOINTS).issubset(joints)
    assert joints["world_to_base"].attrib["type"] == "fixed"
    assert [joints[name].find("axis").attrib["xyz"] for name in EXPECTED_JOINTS] == [
        "0 0 1",
        "0 1 0",
        "0 1 0",
        "1 0 0",
        "0 1 0",
        "1 0 0",
    ]

    for name in EXPECTED_LINKS:
        link = links[name]
        assert link.find("visual") is not None
        assert link.find("collision") is not None
        assert link.find("inertial") is not None


def test_ros2_control_exports_all_joints() -> None:
    robot = generated_robot()
    control = robot.find("ros2_control")
    assert control is not None
    controlled_joints = {joint.attrib["name"]: joint for joint in control.findall("joint")}
    assert set(controlled_joints) == set(EXPECTED_JOINTS)

    for joint in controlled_joints.values():
        assert [item.attrib["name"] for item in joint.findall("command_interface")] == [
            "position"
        ]
        assert [item.attrib["name"] for item in joint.findall("state_interface")] == [
            "position",
            "velocity",
        ]
