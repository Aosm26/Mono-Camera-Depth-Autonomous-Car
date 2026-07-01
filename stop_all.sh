#!/bin/bash

echo "============================================="
echo "  ROS2, Gazebo ve Nav2 Kapatılıyor..."
echo "============================================="

# Kill all ROS2, Nav2, and Gazebo related processes
killall -9 gzserver gzclient robot_state_publisher rviz2 static_transform_publisher controller_server planner_server recoveries_server bt_navigator waypoint_follower lifecycle_manager depthimage_to_laserscan_node ros2 2>/dev/null

echo "Tüm otonom araç simülasyon ve navigasyon süreçleri sonlandırıldı."
