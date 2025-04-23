#!/bin/bash

# Build the ROS2 workspace
colcon build

# Source the setup file
source install/setup.bash

# Launch the ROS2 package
ros2 launch autonomous_rov run_listener_MIR.launch
