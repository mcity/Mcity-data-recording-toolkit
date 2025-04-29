#!/bin/bash

# Republishes rate-limited camera frames for Lucid cameras. Use in conjunction with "lucid-drivers-*.sh"

rate_hz=30

gnome-terminal --tab --title=cam1_rl -- bash -c "ros2 run topic_tools throttle messages /arenacam1/images $rate_hz /cam1_rl/images"
gnome-terminal --tab --title=cam2_rl -- bash -c "ros2 run topic_tools throttle messages /arenacam2/images $rate_hz /cam2_rl/images"
gnome-terminal --tab --title=cam3_rl -- bash -c "ros2 run topic_tools throttle messages /arenacam3/images $rate_hz /cam3_rl/images"
gnome-terminal --tab --title=cam4_rl -- bash -c "ros2 run topic_tools throttle messages /arenacam4/images $rate_hz /cam4_rl/images"
gnome-terminal --tab --title=cam5_rl -- bash -c "ros2 run topic_tools throttle messages /arenacam5/images $rate_hz /cam5_rl/images"
gnome-terminal --tab --title=cam6_rl -- bash -c "ros2 run topic_tools throttle messages /arenacam6/images $rate_hz /cam6_rl/images"

