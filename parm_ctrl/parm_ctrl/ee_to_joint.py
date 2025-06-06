#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
import math

class PoseToJointNode(Node):
    def __init__(self):
        super().__init__('pose_to_joint_node')
        
        # Create service client for inverse kinematics
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Create subscriber for target poses
        self.pose_sub = self.create_subscription(
            Pose,
            'target_pose',
            self.target_pose_callback,
            10)
            
        # Create subscriber for current joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
            
        # Create publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_commands',
            10)
            
        # Initialize current joint states
        self.current_joint_state = None
        
        # Wait for the IK service to be available
        self.get_logger().info('Waiting for /compute_ik service...')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Service available! Waiting for poses...')
    
    def joint_states_callback(self, msg):
        self.current_joint_state = msg
    
    def target_pose_callback(self, pose_msg):
        if self.current_joint_state is None:
            self.get_logger().warn('No joint states received yet, skipping IK')
            return
            
        # Create the service request
        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        
        # Set the group name
        ik_request.group_name = 'arm'
        
        # Set the target pose
        ik_request.pose_stamped.header.frame_id = 'base_link'
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose = pose_msg
        
        # Set current robot state
        robot_state = RobotState()
        robot_state.joint_state = self.current_joint_state
        ik_request.robot_state = robot_state
        
        # Set timeout
        ik_request.timeout.sec = 5
        
        request.ik_request = ik_request
        
        # Send the request asynchronously
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.handle_ik_response)
        
        self.get_logger().info(f'Sent IK request for pose: x={pose_msg.position.x}, y={pose_msg.position.y}, z={pose_msg.position.z}')
    
    def handle_ik_response(self, future):
        try:
            response = future.result()
            
            if response.error_code.val == response.error_code.SUCCESS:
                self.get_logger().info('IK solution found!')
                
                # Create and publish joint command
                joint_command = JointState()
                joint_command.header.stamp = self.get_clock().now().to_msg()
                joint_command.name = response.solution.joint_state.name
                joint_command.position = response.solution.joint_state.position
                
                self.joint_pub.publish(joint_command)
                
                # Log the solution
                for name, angle in zip(joint_command.name, joint_command.position):
                    angle_degrees = math.degrees(angle)
                    self.get_logger().info(f'  {name}: {angle:.4f} rad ({angle_degrees:.2f} deg)')
                
            else:
                self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = PoseToJointNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()