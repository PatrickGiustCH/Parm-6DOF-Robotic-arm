#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys
import termios
import tty
import math
from scipy.spatial.transform import Rotation as R

class KeyboardPoseController(Node):
    def __init__(self):
        super().__init__('keyboard_pose_controller')
        self.publisher_ = self.create_publisher(Pose, 'target_pose', 10)
        self.pose = Pose()

        # Initialize pose
        self.pose.position.x = 0.3
        self.pose.position.y = 0.0
        self.pose.position.z = 0.3
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0

        self.get_logger().info("Keyboard Pose Controller Node Initialized. Use WS for Z, AD for X, QE for Y, IK/JL/UO for rotations.")

    def get_key(self):
        """Reads a single keypress from the keyboard."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def update_orientation(self, roll_delta=0.0, pitch_delta=0.0, yaw_delta=0.0):
        """Update the orientation of the pose using roll, pitch, and yaw deltas."""
        # Convert current quaternion to Euler angles
        current_orientation = [
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w,
        ]
        r = R.from_quat(current_orientation)
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)

        # Apply deltas
        roll += roll_delta
        pitch += pitch_delta
        yaw += yaw_delta

        # Convert back to quaternion
        new_orientation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()
        self.pose.orientation.x = new_orientation[0]
        self.pose.orientation.y = new_orientation[1]
        self.pose.orientation.z = new_orientation[2]
        self.pose.orientation.w = new_orientation[3]

    def run(self):
        """Main loop to read keyboard inputs and update pose."""
        try:
            while rclpy.ok():
                key = self.get_key()

                # Translation controls
                if key == 'w':  # Increase Z
                    self.pose.position.z += 0.05
                elif key == 's':  # Decrease Z
                    self.pose.position.z -= 0.05
                elif key == 'a':  # Decrease X
                    self.pose.position.x -= 0.05
                elif key == 'd':  # Increase X
                    self.pose.position.x += 0.05
                elif key == 'q':  # Decrease Y
                    self.pose.position.y -= 0.05
                elif key == 'e':  # Increase Y
                    self.pose.position.y += 0.05

                # Rotation controls (intuitive around origin axes)
                elif key == 'i':  # Rotate positively around X (roll)
                    self.update_orientation(roll_delta=math.radians(5))
                elif key == 'k':  # Rotate negatively around X (roll)
                    self.update_orientation(roll_delta=-math.radians(5))
                elif key == 'j':  # Rotate negatively around Y (pitch)
                    self.update_orientation(pitch_delta=-math.radians(5))
                elif key == 'l':  # Rotate positively around Y (pitch)
                    self.update_orientation(pitch_delta=math.radians(5))
                elif key == 'u':  # Rotate negatively around Z (yaw)
                    self.update_orientation(yaw_delta=-math.radians(5))
                elif key == 'o':  # Rotate positively around Z (yaw)
                    self.update_orientation(yaw_delta=math.radians(5))

                # Exit
                elif key == '\x03':  # Ctrl+C
                    break

                # Publish updated pose
                self.publisher_.publish(self.pose)
                self.get_logger().info(f"Published Pose: {self.pose}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            self.get_logger().info("Shutting down Keyboard Pose Controller Node.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPoseController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()