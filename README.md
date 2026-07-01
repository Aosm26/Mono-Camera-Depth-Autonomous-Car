# Mono Camera Depth Autonomous Car

This repository contains the ROS2 Foxy workspace packages and configuration scripts for simulating and navigating an autonomous car using a mono camera with depth estimation.

## Workspace Structure

- `src/autonomous_car`: Core ROS2 package.
  - `config/`: Navigation2 parameters (`nav2_params.yaml`).
  - `launch/`: Launch scripts for Gazebo simulation and Nav2 navigation.
  - `urdf/`: Xacro robot description models.
  - `worlds/`: Gazebo obstacle simulation world.
- `setup_pc.sh`: Installation and setup script for Ubuntu 20.04 and ROS2 Foxy.
- `autonomous_car_ws.zip`: Compressed workspace backup.

---

## Installation & Setup

To configure your Ubuntu 20.04 PC with ROS2 Foxy, Gazebo, and Navigation2, execute the setup script:

```bash
chmod +x setup_pc.sh
./setup_pc.sh
```

---

## How to Build and Run

### 1. Build the Workspace
Open a terminal and compile the packages:
```bash
cd ~/autonomous_car_ws # Or your current workspace directory
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Gazebo Simulation
In a terminal (Terminal 1), launch the Gazebo simulation environment:
```bash
ros2 launch autonomous_car sim_launch.py
```

### 3. Launch Navigation2 Stack
In another terminal (Terminal 2), launch the autonomous navigation system:
```bash
source install/setup.bash
ros2 launch autonomous_car nav_launch.py
```

### 4. Sending Goals
Open RViz2 (which automatically opens with the navigation launch) and use the **2D Goal Pose** tool at the top toolbar to send goals to the robot.
