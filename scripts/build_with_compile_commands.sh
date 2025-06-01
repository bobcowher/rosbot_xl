#!/bin/bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Copy the workspace and compile commands json.
cp -pr /ros2_ws/build /ros2_ws_mirror/
cp -pr /ros2_ws/install /ros2_ws_mirror/
cp -pr /ros2_ws/build/compile_commands.json /ros2_ws_mirror/

# Copy the humble installation
cp -pr /opt/ros/humble/* /opt/ros/humble_mirror/
