#!/bin/bash

# ROS2 Camera drivers for LUCID cameras - Without the bayer_rggb16 formatting.

exposure_time=150.0

# Camera 1
gnome-terminal --tab --title=cam1 -- bash -c "ros2 run arena_camera_node start --ros-args -p serial:=\"'243000039'\" -p topic:=/arenacam1/images; /bin/bash"
sleep 3

# Camera 2
gnome-terminal --tab --title=cam2 -- bash -c "ros2 run arena_camera_node start --ros-args -p serial:=\"'224002282'\" -p topic:=/arenacam2/images; /bin/bash"
sleep 3

# Camera 3
gnome-terminal --tab --title=cam3 -- bash -c "ros2 run arena_camera_node start --ros-args -p serial:=\"'223100016'\" -p topic:=/arenacam3/images; /bin/bash"
sleep 3

# Camera 4
gnome-terminal --tab --title=cam4 -- bash -c "ros2 run arena_camera_node start --ros-args -p serial:=\"'224800722'\" -p topic:=/arenacam4/images; /bin/bash"
sleep 3

# Camera 5
gnome-terminal --tab --title=cam5 -- bash -c "ros2 run arena_camera_node start --ros-args -p serial:=\"'224400324'\" -p topic:=/arenacam5/images; /bin/bash"
sleep 3

# Camera 6
gnome-terminal --tab --title=cam6 -- bash -c "ros2 run arena_camera_node start --ros-args -p serial:=\"'224800721'\" -p topic:=/arenacam6/images; /bin/bash"

