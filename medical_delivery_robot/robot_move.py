#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class MoveForward(Node):
    def __init__(self):
        super().__init__('robot_move')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/gazebo_ros_laser_controller/out',
            self.scan_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.scan_data = None
        self.min_safe_distance = 1.2 
        self.speed = 0.5 
        
        self.get_logger().info('Node Move Forward dengan Obstacle Avoidance telah dimulai')
    
    def scan_callback(self, msg):
        """Callback untuk menerima data lidar"""
        self.scan_data = msg
    
    def find_safe_direction(self):
        """Mencari arah yang aman untuk dilalui"""
        if self.scan_data is None:
            return 0.0, True
        
        ranges = np.array(self.scan_data.ranges)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), 
                         self.scan_data.range_max, ranges)
        
        num_readings = len(ranges)
     
        center_idx = num_readings // 2
        front_range = int(num_readings * 20 / 180) 
        front_indices = list(range(center_idx - front_range, center_idx + front_range))
        
        left_indices = list(range(center_idx + front_range, num_readings))
        
        right_indices = list(range(0, center_idx - front_range))
        
        front_min = np.min(ranges[front_indices]) if front_indices else float('inf')
        left_min = np.min(ranges[left_indices]) if left_indices else float('inf')
        right_min = np.min(ranges[right_indices]) if right_indices else float('inf')
        
        self.get_logger().info(
            f'Jarak - Depan: {front_min:.2f}m, Kiri: {left_min:.2f}m, Kanan: {right_min:.2f}m'
        )
        
        if front_min > self.min_safe_distance:
            return 0.0, True
        else:
            if left_min > right_min and left_min > self.min_safe_distance * 0.7:
                self.get_logger().info('Obstacle terdeteksi! Belok kiri')
                return -0.8, False 
            elif right_min > self.min_safe_distance * 0.7:
                self.get_logger().info('Obstacle terdeteksi! Belok kanan')
                return 0.8, False 
            else:
                self.get_logger().warn('Obstacle di semua arah! Berhenti')
                return 0.0, False
        
    def timer_callback(self):
        msg = Twist()
        
        angular_z, can_move_forward = self.find_safe_direction()
        
        if can_move_forward:
            msg.linear.x = -self.speed
            msg.angular.z = angular_z
        elif angular_z != 0.0:
            msg.linear.x = -self.speed * 0.3
            msg.angular.z = angular_z
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    move_forward_node = MoveForward()
    
    try:
        rclpy.spin(move_forward_node)
    except KeyboardInterrupt:
        pass
    
    move_forward_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
