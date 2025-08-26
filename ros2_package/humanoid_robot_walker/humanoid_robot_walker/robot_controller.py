#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
import math
import time

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        
        # Robot state
        self.walking = False
        self.walk_cycle = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Joint names
        self.joint_names = [
            'head_joint', 'left_shoulder', 'right_shoulder',
            'left_hip', 'right_hip', 'left_ankle', 'right_ankle'
        ]
        
        # Timer for animation
        self.timer = self.create_timer(0.05, self.update_joints)  # 20 Hz
        
        self.get_logger().info('Humanoid Robot Controller Started')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands for walking"""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.walking = True
            self.get_logger().info(f'Walking with velocity: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
        else:
            self.walking = False
            self.walk_cycle = 0.0
            
    def goal_callback(self, msg):
        """Handle goal position for navigation"""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.walking = True
        self.get_logger().info(f'New goal: ({self.target_x:.2f}, {self.target_y:.2f})')

    def update_joints(self):
        """Update joint positions for walking animation"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        if self.walking:
            self.walk_cycle += 0.1
            
            # Walking gait - sinusoidal joint movements
            hip_swing = math.sin(self.walk_cycle) * 0.3
            arm_swing = math.sin(self.walk_cycle + math.pi) * 0.2
            
            joint_state.position = [
                0.0,  # head_joint
                arm_swing,  # left_shoulder (opposite to right leg)
                -arm_swing,  # right_shoulder (opposite to left leg)
                hip_swing,  # left_hip
                -hip_swing,  # right_hip
                hip_swing * 0.5,  # left_ankle
                -hip_swing * 0.5   # right_ankle
            ]
            
        else:
            # Standing pose
            joint_state.position = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
