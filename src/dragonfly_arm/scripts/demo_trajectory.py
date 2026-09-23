#!/usr/bin/env python3
"""Small fixed-base simulation motions for the five-axis arm and vector units."""

import sys
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = [f"joint_{index}" for index in range(1, 6)]
TILT_JOINT_NAMES = [
    f"unit_{unit}_{axis}_joint"
    for unit in "abc"
    for axis in ("outer", "inner")
]
ACTION_NAME = "/arm_controller/follow_joint_trajectory"
TILT_ACTION_NAME = "/tilt_controller/follow_joint_trajectory"


class DemoTrajectory(Node):
    def __init__(self):
        super().__init__("demo_trajectory")
        self._arm = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._tilt = ActionClient(self, FollowJointTrajectory, TILT_ACTION_NAME)

    def execute(self, client, names, poses):
        if not client.wait_for_server(timeout_sec=180.0):
            self.get_logger().error(f"Controller unavailable for {names}")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for index, positions in enumerate(poses, start=1):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(names)
            point.time_from_start = Duration(sec=index * 3)
            goal.trajectory.points.append(point)
        goal.goal_time_tolerance = Duration(sec=2)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("Trajectory goal submission timed out.")
            return False
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Trajectory rejected.")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=len(poses) * 3 + 10
        )
        if not result_future.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            self.get_logger().error("Trajectory execution timed out.")
            return False
        wrapped = result_future.result()
        if (wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED
                or wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            self.get_logger().error(f"Trajectory execution failed: {wrapped}")
            return False
        return True

    def run(self):
        # Return to zero between subsystem demonstrations.
        arm_poses = [
            [0.0] * 5,
            [0.20, 0.15, -0.25, 0.10, 0.20],
            [-0.20, -0.15, 0.25, -0.10, -0.20],
            [0.0] * 5,
        ]
        if not self.execute(self._arm, JOINT_NAMES, arm_poses):
            return 2
        self.get_logger().info("Five-axis arm trajectory completed successfully.")
        tilt_poses = [
            [0.0] * 6,
            [0.15, 0.0] * 3,  # outer axes only
            [0.0, 0.15] * 3,  # inner axes only
            [0.15, -0.15, -0.15, 0.15, 0.10, 0.10],
            [0.0] * 6,
        ]
        if not self.execute(self._tilt, TILT_JOINT_NAMES, tilt_poses):
            return 3
        self.get_logger().info("Tilt trajectory completed successfully.")
        self.get_logger().info("Trajectory completed successfully and returned to the home pose.")
        return 0


def main():
    rclpy.init()
    node = DemoTrajectory()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
