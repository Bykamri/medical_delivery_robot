#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class MoveForward(Node):
    def __init__(self):
        super().__init__('robot_move')
        
        # Publisher untuk cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber untuk lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/gazebo_ros_laser_controller/out',
            self.scan_callback,
            10
        )
        
        # Timer untuk publish velocity secara berkala
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Variabel untuk menyimpan data lidar
        self.scan_data = None
        self.min_safe_distance = 1.5  # Jarak aman minimum (meter)
        self.speed = 0.5  # Kecepatan default
        
        self.get_logger().info('Node Move Forward dengan Obstacle Avoidance telah dimulai')
    
    def scan_callback(self, msg):
        """Callback untuk menerima data lidar"""
        self.scan_data = msg
    
    def find_safe_direction(self):
        """Mencari arah yang aman untuk dilalui"""
        if self.scan_data is None:
            return 0.0, True  # Tidak ada data, jalan terus
        
        ranges = np.array(self.scan_data.ranges)
        # Ganti inf dan nan dengan nilai maksimal
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), 
                         self.scan_data.range_max, ranges)
        
        num_readings = len(ranges)
        
        # Lidar scan 180 derajat: index 0 = kanan (-90°), tengah = depan (0°), max = kiri (90°)
        # Dengan min_angle=-1.57 dan max_angle=1.57
        
        # Depan: tengah ±20 derajat (sekitar 40° total)
        center_idx = num_readings // 2
        front_range = int(num_readings * 20 / 180)  # 20 derajat dari 180
        front_indices = list(range(center_idx - front_range, center_idx + front_range))
        
        # Kiri: dari tengah+20° sampai akhir (sekitar 70°)
        left_indices = list(range(center_idx + front_range, num_readings))
        
        # Kanan: dari awal sampai tengah-20° (sekitar 70°)
        right_indices = list(range(0, center_idx - front_range))
        
        # Hitung jarak minimum di setiap sektor
        front_min = np.min(ranges[front_indices]) if front_indices else float('inf')
        left_min = np.min(ranges[left_indices]) if left_indices else float('inf')
        right_min = np.min(ranges[right_indices]) if right_indices else float('inf')
        
        self.get_logger().info(
            f'Jarak - Depan: {front_min:.2f}m, Kiri: {left_min:.2f}m, Kanan: {right_min:.2f}m'
        )
        
        # Logika pengambilan keputusan
        if front_min > self.min_safe_distance:
            # Jalan depan aman, lurus
            return 0.0, True
        else:
            # Ada obstacle di depan, cari jalur alternatif
            if left_min > right_min and left_min > self.min_safe_distance * 0.7:
                # Belok kiri
                self.get_logger().info('Obstacle terdeteksi! Belok kiri')
                return -0.8, False  # Angular velocity negatif = belok kiri
            elif right_min > self.min_safe_distance * 0.7:
                # Belok kanan
                self.get_logger().info('Obstacle terdeteksi! Belok kanan')
                return 0.8, False  # Angular velocity positif = belok kanan
            else:
                # Semua arah berbahaya, berhenti atau mundur
                self.get_logger().warn('Obstacle di semua arah! Berhenti')
                return 0.0, False
        
    def timer_callback(self):
        """Callback timer untuk publish velocity"""
        msg = Twist()
        
        # Dapatkan arah yang aman
        angular_z, can_move_forward = self.find_safe_direction()
        
        if can_move_forward:
            # Jalan lurus
            msg.linear.x = -self.speed
            msg.angular.z = angular_z
        elif angular_z != 0.0:
            # Belok untuk menghindari obstacle
            msg.linear.x = -self.speed * 0.3  # Pelan saat belok
            msg.angular.z = angular_z
        else:
            # Berhenti
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        
        # Publish pesan
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    move_forward_node = MoveForward()
    
    try:
        rclpy.spin(move_forward_node)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    move_forward_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
