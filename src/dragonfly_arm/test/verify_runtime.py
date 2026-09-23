#!/usr/bin/env python3
"""Bounded runtime readiness check without the ros2 CLI discovery daemon."""
import math
import time
from collections import deque

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node("verify_dragonfly_runtime")
    expected = {f"joint_{i}" for i in range(1, 6)} | {
        f"unit_{unit}_{axis}_joint" for unit in "abc" for axis in ("outer", "inner")
    }
    latest = deque(maxlen=1)
    sub = node.create_subscription(JointState, "/joint_states", latest.append, qos_profile_sensor_data)
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    deadline = time.monotonic() + 180
    future = None
    active = set()
    next_query = 0.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if future is not None and future.done():
                result = future.result()
                active = {c.name for c in result.controller if c.state == "active"}
                future = None
            if latest:
                msg = latest[0]
                indices = {name: i for i, name in enumerate(msg.name)}
                valid = expected.issubset(indices) and all(
                    indices[name] < len(msg.position) and indices[name] < len(msg.velocity)
                    and math.isfinite(msg.position[indices[name]])
                    and math.isfinite(msg.velocity[indices[name]]) for name in expected
                )
                if valid and {"joint_state_broadcaster", "arm_controller", "tilt_controller"}.issubset(active):
                    print("All three controllers active; 11 finite joint position/velocity states received.")
                    return
            if future is None and time.monotonic() >= next_query and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
                next_query = time.monotonic() + 0.5
        raise RuntimeError(f"Runtime not ready within 180 s; active controllers: {sorted(active)}")
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
