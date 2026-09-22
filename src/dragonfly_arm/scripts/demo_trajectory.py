#!/usr/bin/env python3
"""Send one safe, repeatable trajectory to the Dragonfly arm controller."""

import sys

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
ACTION_NAME = "/arm_controller/follow_joint_trajectory"


class DemoTrajectory(Node):
    """Wait for the controller and execute the demonstration trajectory."""

    def __init__(self) -> None:
        super().__init__("demo_trajectory")
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    @staticmethod
    def _point(positions: list[float], seconds: int) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=seconds)
        return point

    def run(self) -> int:
        self.get_logger().info(f"Waiting for {ACTION_NAME} ...")
        if not self._client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("Trajectory action server was not available after 60 seconds.")
            return 2

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        home = [0.0, -0.45, 0.80, 0.0, 0.35, 0.0]
        goal.trajectory.points = [
            self._point(home, 3),
            self._point([0.75, -0.80, 1.10, 0.55, 0.65, -0.45], 7),
            self._point([-0.75, -0.30, 0.55, -0.55, -0.40, 0.70], 11),
            self._point(home, 15),
        ]
        goal.goal_time_tolerance = Duration(sec=2)

        self.get_logger().info("Sending four-point trajectory ...")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error("Timed out while sending the trajectory goal.")
            return 3

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("The arm controller rejected the trajectory.")
            return 4

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=25.0)
        if not result_future.done():
            self.get_logger().error("Trajectory execution timed out; requesting cancellation.")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return 5

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("The trajectory action returned no result.")
            return 6

        result = wrapped_result.result
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error(
                "Trajectory failed: status=%d, error_code=%d, message=%s"
                % (wrapped_result.status, result.error_code, result.error_string)
            )
            return 7

        self.get_logger().info("Trajectory completed successfully and returned to the home pose.")
        return 0


def main() -> None:
    rclpy.init()
    node = DemoTrajectory()
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("Trajectory demonstration interrupted.")
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
