#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class MoveForwardEnhanced(Node):
    def __init__(self):
        super().__init__('robot_move_enhanced')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.scan_data = None
        self.min_safe_distance = 1.2 
        self.speed = 0.5 
        self.rotation_speed = 0.6
        self.backup_speed = 0.3
        
        # State machine untuk backup dan rotasi
        self.state = 'normal'  # 'normal', 'backing_up', 'rotating'
        self.backup_counter = 0
        self.max_backup_time = 15  # 1.5 detik mundur
        self.rotation_counter = 0
        self.max_rotation_time = 60  # 6 detik rotasi (fallback timeout)
        self.rotation_direction = 0.0
        
        self.get_logger().info('Node Move Forward Enhanced dengan Backup dan Rotasi telah dimulai')
    
    def scan_callback(self, msg):
        """Callback untuk menerima data lidar"""
        self.scan_data = msg
    
    def find_safe_direction(self):
        """Mencari arah yang aman untuk dilalui"""
        if self.scan_data is None:
            return 0.0, True, 'forward'
        
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
            return 0.0, True, 'forward'
        else:
            if left_min > right_min and left_min > self.min_safe_distance * 0.7:
                self.get_logger().info('Obstacle terdeteksi! Belok kiri')
                return -0.8, False, 'turn_left'
            elif right_min > self.min_safe_distance * 0.7:
                self.get_logger().info('Obstacle terdeteksi! Belok kanan')
                return 0.8, False, 'turn_right'
            else:
                # Obstacle di semua arah - perlu backup dan rotasi
                self.get_logger().warn('Obstacle di semua arah! Perlu backup')
                # Tentukan arah rotasi berdasarkan sisi yang lebih jauh
                if left_min > right_min:
                    return -self.rotation_speed, False, 'backup_rotate_left'
                else:
                    return self.rotation_speed, False, 'backup_rotate_right'
        
    def timer_callback(self):
        msg = Twist()
        
        # State machine logic
        if self.state == 'backing_up':
            # Proses mundur
            self.backup_counter += 1
            msg.linear.x = self.backup_speed  # Positif untuk mundur
            msg.angular.z = 0.0
            
            self.get_logger().info(f'Mundur... ({self.backup_counter}/{self.max_backup_time})')
            
            if self.backup_counter >= self.max_backup_time:
                # Selesai mundur, mulai rotasi
                self.state = 'rotating'
                self.backup_counter = 0
                self.rotation_counter = 0
                self.get_logger().info(f'Selesai mundur, mulai memutar ke arah yang lebih aman')
        
        elif self.state == 'rotating':
            # Proses rotasi - cek jarak depan saat memutar
            self.rotation_counter += 1
            
            # Cek jarak depan saat memutar
            if self.scan_data is not None:
                ranges = np.array(self.scan_data.ranges)
                ranges = np.where(np.isinf(ranges) | np.isnan(ranges), 
                                 self.scan_data.range_max, ranges)
                
                num_readings = len(ranges)
                center_idx = num_readings // 2
                front_range = int(num_readings * 20 / 180)
                front_indices = list(range(center_idx - front_range, center_idx + front_range))
                front_min = np.min(ranges[front_indices]) if front_indices else float('inf')
                
                # Jika menemukan ruang aman di depan, berhenti memutar
                if front_min > self.min_safe_distance:
                    self.state = 'normal'
                    self.rotation_counter = 0
                    self.get_logger().info(f'Ruang aman ditemukan! Jarak depan: {front_min:.2f}m')
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    # Terus memutar
                    msg.linear.x = 0.0
                    msg.angular.z = self.rotation_direction
                    self.get_logger().info(f'Memutar mencari ruang aman... Jarak depan: {front_min:.2f}m ({self.rotation_counter}/{self.max_rotation_time})')
            else:
                msg.linear.x = 0.0
                msg.angular.z = self.rotation_direction
                self.get_logger().info(f'Memutar... ({self.rotation_counter}/{self.max_rotation_time})')
            
            # Fallback: jika sudah memutar terlalu lama, paksa kembali ke normal
            if self.rotation_counter >= self.max_rotation_time:
                self.state = 'normal'
                self.rotation_counter = 0
                self.get_logger().warn('Timeout rotasi! Kembali ke mode normal')
        
        else:  # state == 'normal'
            angular_z, can_move_forward, action = self.find_safe_direction()
            
            if action == 'backup_rotate_left' or action == 'backup_rotate_right':
                # Mulai proses backup dan rotasi
                self.state = 'backing_up'
                self.backup_counter = 0
                self.rotation_direction = angular_z
                msg.linear.x = self.backup_speed
                msg.angular.z = 0.0
                self.get_logger().warn('Memulai backup karena obstacle di semua arah!')
            
            elif can_move_forward:
                # Jalan maju normal
                msg.linear.x = -self.speed
                msg.angular.z = angular_z
            
            elif angular_z != 0.0:
                # Belok dengan kecepatan maju dikurangi
                msg.linear.x = -self.speed * 0.3
                msg.angular.z = angular_z
            
            else:
                # Berhenti
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    move_forward_node = MoveForwardEnhanced()
    
    try:
        rclpy.spin(move_forward_node)
    except KeyboardInterrupt:
        pass
    
    move_forward_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
