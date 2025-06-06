#!/usr/bin/env python3
"""
pose_to_joint_traj_node.py
──────────────────────────
Subscribe to an end-effector pose goal, call MoveIt for planning + retiming,
and publish the resulting JointTrajectory to the robot driver.

Requirements
------------
• ROS 2 (tested on Jazzy)
• moveit_py  (comes with MoveIt 2 ≥ humble)
• move_group already running (e.g. via your move_group.launch.py)
"""

from __future__ import annotations
import rclpy, sys
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_py.interfaces import MoveGroupInterface
from moveit.core.robot_trajectory import RobotTrajectory    # pybind11 binding
from moveit_msgs.msg import RobotTrajectory as RobotTrajMsg

class PoseToJointTraj(Node):
    """Converts EE poses to MoveIt-planned JointTrajectories."""

    def __init__(self):
        super().__init__("pose_to_joint_traj")

        # ┌─────────────────────────────────────────────┐
        # │ Parameters you might tweak at launch time   │
        # └─────────────────────────────────────────────┘
        self.declare_parameter("group_name",   "manipulator")
        self.declare_parameter("eef_link",     "tool0")
        self.declare_parameter("vel_scale",     0.2)   # 0‒1, overridable in RViz
        self.declare_parameter("cmd_topic",   "/joint_commands")
        self.declare_parameter("pose_topic",  "/desired_eef_pose")

        g = self.get_parameter
        self._velocity_scale = g("vel_scale").value

        # MoveIt Python interface (needs move_group already running)
        self._mgi = MoveGroupInterface(
            node=self,
            group_name=g("group_name").value,
            callback_group=None,          # rclpy default
        )
        self._mgi.set_end_effector_link(g("eef_link").value)

        # Communications
        self._cmd_pub = self.create_publisher(
            JointTrajectory, g("cmd_topic").value, 10
        )
        self.create_subscription(
            PoseStamped, g("pose_topic").value, self._pose_cb, 10
        )

        self.get_logger().info(
            f"Pose-to-trajectory bridge ready → publish poses on "
            f"{g('pose_topic').value}"
        )

    # ─────────────────────────────────────────────────────────────────────
    # Convert an incoming pose into a planned + time-parameterised trajectory
    # ─────────────────────────────────────────────────────────────────────
    def _pose_cb(self, pose: PoseStamped) -> None:
        self.get_logger().info(
            f"Planning to pose x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}"
        )

        self._mgi.set_pose_target(pose)
        self._mgi.set_max_velocity_scaling_factor(self._velocity_scale)

        success, plan_msg = self._mgi.plan()
        if not success or not plan_msg:
            self.get_logger().warn("⚠️  Planning failed")
            return

        # ------------------------------------------------------------------
        # Optional but recommended: re-time with Ruckig/TOTG for smoother &
        # exactly-scaled motion (works in Python via pybind11 bindings)
        # ------------------------------------------------------------------
        traj = RobotTrajectory(
            self._mgi.get_robot_model(), self._mgi.get_joint_model_group()
        )
        traj.from_msg(plan_msg.trajectory)
        traj.apply_ruckig_smoothing(
            velocity_scaling_factor=self._velocity_scale,
            acceleration_scaling_factor=self._velocity_scale,
        )
        retimed_msg: RobotTrajMsg = traj.to_msg()

        # Publish to the driver
        self._cmd_pub.publish(retimed_msg.joint_trajectory)
        self.get_logger().info(
            f"Trajectory with {len(retimed_msg.joint_trajectory.points)} points "
            f"sent to {self.get_parameter('cmd_topic').value}"
        )


def main() -> None:
    rclpy.init()
    node = PoseToJointTraj()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
