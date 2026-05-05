#!/bin/bash

# ROS2 Camera drivers for LUCID cameras - Autoexposure
# Documentation: https://github.com/lucidvisionlabs/arena_camera_ros2/

pixelformat="bayer_rggb8" # Default, faster?
#pixelformat="rgb8"
#pixelformat="bayer_rggb16"

gamma=0.5
target_brightness=70

# Camera 1
serial="243000039"
gnome-terminal --tab --title=cam1 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p gamma:=$gamma -p target_brightness:=$target_brightness -p serial:=$serial -p topic:=/arenacam1/images -p camera_name:=arenacam1; /bin/bash"
sleep 3

# Camera 2
serial="224002282"
gnome-terminal --tab --title=cam2 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p gamma:=$gamma -p target_brightness:=$target_brightness -p serial:=$serial -p topic:=/arenacam2/images -p camera_name:=arenacam2; /bin/bash"
sleep 3000

# Camera 3
serial="223100016"
gnome-terminal --tab --title=cam3 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p gamma:=$gamma -p target_brightness:=$target_brightness -p serial:=$serial -p topic:=/arenacam3/images -p camera_name:=arenacam3; /bin/bash"
#sleep 3

# Camera 4
serial="224800722"
gnome-terminal --tab --title=cam4 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p gamma:=$gamma -p target_brightness:=$target_brightness -p serial:=$serial -p topic:=/arenacam4/images -p camera_name:=arenacam4; /bin/bash"
sleep 3

# Camera 5
serial="224400324"
gnome-terminal --tab --title=cam5 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p gamma:=$gamma -p target_brightness:=$target_brightness -p serial:=$serial -p topic:=/arenacam5/images -p camera_name:=arenacam5; /bin/bash"
sleep 3

# Camera 6
serial="224800721"
gnome-terminal --tab --title=cam6 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p gamma:=$gamma -p target_brightness:=$target_brightness -p serial:=$serial -p topic:=/arenacam6/images -p camera_name:=arenacam6; /bin/bash"

