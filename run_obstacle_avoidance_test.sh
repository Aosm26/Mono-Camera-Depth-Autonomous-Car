#!/bin/bash

# Clear Anaconda from PATH for this script session to avoid python version mismatch with ROS2
export PATH=$(echo $PATH | sed -e 's|/home/aosm/anaconda3/bin:||' -e 's|/home/aosm/anaconda3/condabin:||')

# Color codes for pretty printing
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}====================================================${NC}"
echo -e "${BLUE}   ROS2 & Gazebo & Rosbridge & Obstacle Avoidance   ${NC}"
echo -e "${BLUE}====================================================${NC}"

# Source ROS2 and workspace
echo "Sourcing ROS2 and workspace..."
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Array to keep track of background PIDs
PIDS=()

# Function to clean up background processes on exit
cleanup() {
    echo -e "\n${RED}Shutting down all processes gracefully...${NC}"
    
    # Send SIGINT (Ctrl+C) to all child processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null
        fi
    done
    
    # Wait briefly
    sleep 2
    
    # Force kill any remaining processes if necessary
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null
        fi
    done
    
    # Also run stop_all.sh to ensure Gazebo server/client are terminated
    if [ -f "./stop_all.sh" ]; then
        ./stop_all.sh >/dev/null 2>&1
    fi
    
    echo -e "${GREEN}All processes terminated. Goodbye!${NC}"
    exit 0
}

# Trap Ctrl+C (SIGINT) and SIGTERM
trap cleanup SIGINT SIGTERM

# 1. Start Gazebo Simulation
echo -e "${GREEN}[1/4] Starting Gazebo Simulation (logging to simulation.log)...${NC}"
ros2 launch autonomous_car sim_launch.py > simulation.log 2>&1 &
PIDS+=($!)
sleep 5 # Wait for Gazebo to initialize

# 2. Start Rosbridge Server
echo -e "${GREEN}[2/4] Starting Rosbridge Server (logging to rosbridge.log)...${NC}"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > rosbridge.log 2>&1 &
PIDS+=($!)
sleep 2

# 3. Start PC Image Compressor Node
echo -e "${GREEN}[3/4] Starting Image Compressor Node (logging to compressor.log)...${NC}"
/usr/bin/python3 image_compressor.py > compressor.log 2>&1 &
PIDS+=($!)
sleep 1

# 4. Start PC Obstacle Avoidance Controller
echo -e "${GREEN}[4/4] Starting Obstacle Avoidance Controller...${NC}\n"
# Run this in the foreground so the user can see the logs directly in the terminal
/usr/bin/python3 pc_obstacle_avoidance.py

# If pc_obstacle_avoidance.py exits, trigger cleanup
cleanup
