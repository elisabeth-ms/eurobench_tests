#!/bin/bash
# Basic entrypoint for ROS / Catkin Docker containers

# Source ROS and Catkin workspaces
source /opt/ros/noetic/setup.bash

if [ -f /catkin_ws/devel/setup.bash ]
then
  echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
  source /catkin_ws/devel/setup.bash
fi
echo "Sourced Catkin workspace!"

# Execute the command passed into this entrypoint
exec "$@"