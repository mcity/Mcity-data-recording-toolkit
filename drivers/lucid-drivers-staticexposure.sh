#!/bin/bash

# ROS2 Camera drivers for LUCID cameras - Static exposure

exposure_time=90.0
#pixelformat="bayer_rggb8" # Default, faster?
#pixelformat="rgb8"
pixelformat="bayer_rggb16"

# Camera 1
gnome-terminal --tab --title=cam1 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p exposure_time:=$exposure_time -p serial:=\"'243000039'\" -p topic:=/arenacam1/images -p camera_name:=arenacam1; /bin/bash"
sleep 3

# Camera 2
gnome-terminal --tab --title=cam2 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p exposure_time:=$exposure_time -p serial:=\"'224002282'\" -p topic:=/arenacam2/images -p camera_name:=arenacam2; /bin/bash"
sleep 3

# Camera 3
gnome-terminal --tab --title=cam3 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p exposure_time:=$exposure_time -p serial:=\"'223100016'\" -p topic:=/arenacam3/images -p camera_name:=arenacam3; /bin/bash"
sleep 3

# Camera 4
gnome-terminal --tab --title=cam4 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p exposure_time:=$exposure_time -p serial:=\"'224800722'\" -p topic:=/arenacam4/images -p camera_name:=arenacam4; /bin/bash"
sleep 3

# Camera 5
gnome-terminal --tab --title=cam5 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p exposure_time:=$exposure_time -p serial:=\"'224400324'\" -p topic:=/arenacam5/images -p camera_name:=arenacam5; /bin/bash"
sleep 3

# Camera 6
gnome-terminal --tab --title=cam6 -- bash -c "ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p exposure_time:=$exposure_time -p serial:=\"'224800721'\" -p topic:=/arenacam6/images -p camera_name:=arenacam6; /bin/bash"

