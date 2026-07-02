#!/usr/bin/env python3
"""
PC Obstacle Avoidance Controller Node
Subscribes to '/scan' (published by Raspberry Pi or simulator) and decides 
movement commands (Forward, Left, Right, Stop), printing them to the terminal 
and publishing cmd_vel commands to the simulated car in Gazebo.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

# ANSI Colors for beautiful terminal output
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class PcObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('pc_obstacle_avoidance')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Obstacle avoidance parameters
        self.safe_distance = 2.5  # Threshold in meters
        self.linear_speed = 0.8   # Forward speed (m/s)
        self.angular_speed = 0.8  # Turning speed (rad/s)
        
        self.get_logger().info(f"{Colors.OKGREEN}{Colors.BOLD}PC Obstacle Avoidance Node started.{Colors.ENDC}")
        self.get_logger().info(f"Subscribed to /scan, Publishing to /cmd_vel")
        print("="*60)
        print(f"{Colors.BOLD}   Otonom Araç Engel Kaçınma Test Arayüzü{Colors.ENDC}")
        print("="*60)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Replace inf and nan values with max_range
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), msg.range_max, ranges)
        
        num_readings = len(ranges)
        if num_readings == 0:
            return
            
        # Split scan into three main sectors: Left, Center, Right
        # In rpi_obstacle_detector.py, index 0 is left and index N-1 is right
        third = num_readings // 3
        left_sector = ranges[0:third]
        center_sector = ranges[third:2*third]
        right_sector = ranges[2*third:]
        
        # Calculate minimum distance in each sector
        min_left = float(np.min(left_sector))
        min_center = float(np.min(center_sector))
        min_right = float(np.min(right_sector))
        
        # Determine movement command
        twist = Twist()
        direction_msg = ""
        
        if min_center < self.safe_distance:
            # Obstacle detected in front!
            if min_left > min_right:
                # Left is clearer, turn Left
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
                direction_msg = f"{Colors.WARNING}⚠️ ENGEL VAR! -> [SOLA DÖN] (Sol: {min_left:.2f}m, Ön: {min_center:.2f}m, Sağ: {min_right:.2f}m){Colors.ENDC}"
            elif min_right > min_left:
                # Right is clearer, turn Right
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
                direction_msg = f"{Colors.WARNING}⚠️ ENGEL VAR! -> [SAĞA DÖN] (Sol: {min_left:.2f}m, Ön: {min_center:.2f}m, Sağ: {min_right:.2f}m){Colors.ENDC}"
            else:
                # Both blocked, Stop and turn in place
                twist.linear.x = -0.05  # Slight backup
                twist.angular.z = self.angular_speed
                direction_msg = f"{Colors.FAIL}🚨 HER YER TIKALI! -> [GERİ & DÖN] (Sol: {min_left:.2f}m, Ön: {min_center:.2f}m, Sağ: {min_right:.2f}m){Colors.ENDC}"
        else:
            # Path is clear, go forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            direction_msg = f"{Colors.OKGREEN}✅ YOL AÇIK -> [İLERİ GİT] (Sol: {min_left:.2f}m, Ön: {min_center:.2f}m, Sağ: {min_right:.2f}m){Colors.ENDC}"
            
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        
        # Print status to terminal
        print(direction_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PcObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{Colors.OKBLUE}Durduruluyor...{Colors.ENDC}")
    finally:
        # Stop the robot on shutdown
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
