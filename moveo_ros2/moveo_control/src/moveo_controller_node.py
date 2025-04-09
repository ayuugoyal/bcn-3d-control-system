#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import serial
import time
import math
import numpy as np
import tf_transformations

class MoveoController(Node):
    def __init__(self):
        super().__init__('moveo_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # Initialize serial connection to Arduino
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.ser = None
        
        # Joint state publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Cartesian pose publisher
        self.cartesian_pose_pub = self.create_publisher(
            PoseStamped,
            '/moveo/cartesian_pose',
            10
        )
        
        # Joint command subscriber
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/moveo/joint_command',
            self.joint_command_callback,
            10
        )
        
        # Cartesian command subscriber
        self.cartesian_cmd_sub = self.create_subscription(
            PoseStamped,
            '/moveo/cartesian_command',
            self.cartesian_command_callback,
            10
        )
        
        # Gripper command subscriber
        self.gripper_cmd_sub = self.create_subscription(
            Float32,
            '/moveo/gripper_command',
            self.gripper_command_callback,
            10
        )
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Initialize joint positions
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Last one is gripper
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']
        
        self.get_logger().info('Moveo controller initialized')
    
    def publish_joint_states(self):
        # Read joint positions from Arduino if available
        if self.ser is not None and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('JOINTS:'):
                    # Parse joint positions from Arduino
                    parts = line.split(':')[1].split(',')
                    if len(parts) >= 6:
                        for i in range(6):
                            self.joint_positions[i] = float(parts[i])
            except Exception as e:
                self.get_logger().error(f'Error reading from Arduino: {e}')
        
        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        joint_state_msg.velocity = [0.0] * 6
        joint_state_msg.effort = [0.0] * 6
        
        self.joint_state_pub.publish(joint_state_msg)
        
        # Calculate and publish cartesian pose using forward kinematics
        cartesian_pose = self.forward_kinematics(self.joint_positions[:5])
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = cartesian_pose[0]
        pose_msg.pose.position.y = cartesian_pose[1]
        pose_msg.pose.position.z = cartesian_pose[2]
        
        # Convert roll, pitch, yaw to quaternion
        quaternion = tf_transformations.quaternion_from_euler(
            cartesian_pose[3], cartesian_pose[4], cartesian_pose[5]
        )
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        self.cartesian_pose_pub.publish(pose_msg)
    
    def joint_command_callback(self, msg):
        # Process joint command
        if self.ser is None:
            self.get_logger().warn('Cannot send command: Arduino not connected')
            return
        
        # Extract joint positions from message
        positions = []
        for name in self.joint_names[:5]:  # Exclude gripper
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            else:
                # Use current position if not specified
                idx = self.joint_names.index(name)
                positions.append(self.joint_positions[idx])
        
        # Send command to Arduino
        command = f"J:{positions[0]},{positions[1]},{positions[2]},{positions[3]},{positions[4]}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent joint command: {command}')
    
    def cartesian_command_callback(self, msg):
        # Process cartesian command using inverse kinematics
        if self.ser is None:
            self.get_logger().warn('Cannot send command: Arduino not connected')
            return
        
        # Extract position and orientation
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Convert quaternion to Euler angles
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        
        # Calculate joint angles using inverse kinematics
        joint_angles = self.inverse_kinematics(x, y, z, roll, pitch, yaw)
        
        # Send command to Arduino
        command = f"J:{joint_angles[0]},{joint_angles[1]},{joint_angles[2]},{joint_angles[3]},{joint_angles[4]}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent cartesian command: {command}')
    
    def gripper_command_callback(self, msg):
        # Process gripper command
        if self.ser is None:
            self.get_logger().warn('Cannot send command: Arduino not connected')
            return
        
        # Extract gripper position (0-100%)
        gripper_pos = msg.data
        
        # Send command to Arduino
        command = f"G:{gripper_pos}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent gripper command: {command}')
    
    def forward_kinematics(self, joint_angles):
        # Simplified forward kinematics for BCN3D Moveo
        # This is a placeholder - you would implement the actual kinematics based on your robot's dimensions
        
        # Convert joint angles from degrees to radians
        j1 = math.radians(joint_angles[0])
        j2 = math.radians(joint_angles[1])
        j3 = math.radians(joint_angles[2])
        j4 = math.radians(joint_angles[3])
        j5 = math.radians(joint_angles[4])
        
        # Example calculation (simplified)
        # In a real implementation, you would use the DH parameters of your robot
        l1 = 0.10  # Length of link 1 in meters
        l2 = 0.15  # Length of link 2 in meters
        l3 = 0.12  # Length of link 3 in meters
        l4 = 0.08  # Length of link 4 in meters
        
        # Simplified calculation for end effector position
        x = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3) + l4 * math.cos(j2 + j3 + j4)) * math.cos(j1)
        y = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3) + l4 * math.cos(j2 + j3 + j4)) * math.sin(j1)
        z = l1 + l2 * math.sin(j2) + l3 * math.sin(j2 + j3) + l4 * math.sin(j2 + j3 + j4)
        
        # End effector orientation (simplified)
        roll = j5
        pitch = j2 + j3 + j4
        yaw = j1
        
        return [x, y, z, roll, pitch, yaw]
    
    def inverse_kinematics(self, x, y, z, roll, pitch, yaw):
        # Simplified inverse kinematics for BCN3D Moveo
        # This is a placeholder - you would implement the actual kinematics based on your robot's dimensions
        
        # Example calculation (simplified)
        # In a real implementation, you would use the DH parameters of your robot
        l1 = 0.10  # Length of link 1 in meters
        l2 = 0.15  # Length of link 2 in meters
        l3 = 0.12  # Length of link 3 in meters
        l4 = 0.08  # Length of link 4 in meters
        
        # Simplified calculation for joint angles
        j1 = math.atan2(y, x)  # Base rotation
        
        # Distance from base to end effector in XY plane
        r = math.sqrt(x*x + y*y)
        
        # Height from base to end effector
        h = z - l1
        
        # Distance from shoulder to wrist
        d = math.sqrt(r*r + h*h)
        
        # Angles for shoulder and elbow
        a1 = math.atan2(h, r)
        a2 = math.acos((l2*l2 + d*d - l3*l3 - l4*l4) / (2 * l2 * d))
        j2 = a1 + a2
        
        a3 = math.acos((l2*l2 + l3*l3 + l4*l4 - d*d) / (2 * l2 * (l3 + l4)))
        j3 = math.pi - a3
        
        # Wrist angle to achieve desired orientation
        j4 = pitch - j2 - j3
        
        # Roll of the end effector
        j5 = roll
        
        # Convert to degrees
        j1_deg = math.degrees(j1)
        j2_deg = math.degrees(j2)
        j3_deg = math.degrees(j3)
        j4_deg = math.degrees(j4)
        j5_deg = math.degrees(j5)
        
        return [j1_deg, j2_deg, j3_deg, j4_deg, j5_deg]

def main(args=None):
    rclpy.init(args=args)
    node = MoveoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
