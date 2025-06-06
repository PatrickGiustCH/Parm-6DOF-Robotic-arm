#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

class HardwareEmulator(Node):
    def __init__(self):
        super().__init__('hardware_emulator')

        # Subscriber to read joint commands
        self.subscription = self.create_subscription(
            JointState,
            'joint_commands',
            self.joint_command_callback,
            10
        )

        # Publisher to publish joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Service to handle "home" action
        self.home_service = self.create_service(Trigger, 'home', self.handle_home)

        # Initialize joint state
        self.joint_state = JointState()
        self.joint_state.name = [
            "revolute_1", "revolute_2", "revolute_3",
            "revolute_4", "revolute_5", "revolute_6",
            "prismatic_1", "prismatic_2"
        ]
        self.joint_state.position = [0.0] * len(self.joint_state.name)

        # Timer to publish joint states at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_joint_state)

        self.get_logger().info("Hardware Emulator Node Initialized. Listening to 'joint_commands', publishing to 'joint_states' at 10 Hz, and handling 'home' service.")

    def joint_command_callback(self, msg):
        """Callback to handle incoming joint commands and update joint states."""
        self.get_logger().info(f"Received joint command: {msg}")
        self.joint_state = msg  # Update the joint state

    def handle_home(self, request, response):
        """Service callback to handle the 'home' action."""
        self.get_logger().info("Home service triggered. Setting all joint positions to zero.")
        self.joint_state.position = [0.0] * len(self.joint_state.name)  # Reset all joint positions to zero
        response.success = True
        response.message = "All joints have been set to zero."
        return response

    def publish_joint_state(self):
        """Publish the current joint state at 10 Hz."""
        # Add a timestamped header
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()