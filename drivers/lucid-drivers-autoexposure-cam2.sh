#!/bin/bash

# ROS2 Camera drivers for LUCID cameras - Autoexposure, camera 2 only
# Documentation: https://github.com/lucidvisionlabs/arena_camera_ros2/

pixelformat="bayer_rggb8" # Default, faster?
#pixelformat="rgb8"
#pixelformat="bayer_rggb16"

# Camera 2
#serial="224002282"
#gnome-terminal --tab --title=cam2 -- 
#bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p serial:=$serial -p topic:=/arenacam2/images -p camera_name:=arenacam2; /bin/bash"


# Camera 6
serial="224800721"
#gnome-terminal --tab --title=cam2 -- 
bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p serial:=$serial -p topic:=/arenacam6/images -p camera_name:=arenacam6; /bin/bash"

