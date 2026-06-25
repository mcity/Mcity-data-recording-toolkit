#!/bin/bash

# ROS2 Camera drivers for LUCID cameras - PTP + Scheduled Action Command sync.
# Launches the renamed driver package "arena_camera_node_trigger" with
# action_trigger_mode enabled on all six cameras. Exactly ONE camera (cam1) is
# the action master: it owns the /trigger_all service and broadcasts the
# scheduled GigE Vision action command that fires every camera simultaneously.
#
# Documentation: see Mcity-data-recording-toolkit/trigger_mode/README.md and
# arena_camera_ros2/trigger_mode/README.md
#
# Prereq: build the renamed package on the vehicle:
#   colcon build --packages-select arena_camera_node_trigger && source install/setup.bash

##### CONFIG #####
pixelformat="bayer_rggb8" # Default, faster
#pixelformat="rgb8"
#pixelformat="bayer_rggb16"

gamma=0.5

# Fixed exposure (microseconds) is recommended for deterministic synchronized
# capture under triggering. [verify] tune this for the rig/scene on the vehicle.
exposure_time=4000

# Shared action keys/mask -- MUST be identical on all six cameras and on the
# fire command. Defaults match the driver defaults.
action_device_key=1
action_group_key=1
action_group_mask=1

# Margin (seconds) added ahead of the scheduled execute time to clear worst-case
# network latency/jitter.
action_lead_time=0.05

# PTP domain. Cameras commonly assume 0; must match RTK grandmaster + switch.
ptp_domain=0

# Optional in-driver continuous rate (Hz) on the master. 0 = one-shot only
# (fire via ./trigger-all.sh). For a hardware-precise steady rate set e.g. 5.0.
action_trigger_rate=0.0

##################

# Common args for every camera (action_trigger_mode on, shared keys).
common_args="-p pixelformat:=$pixelformat -p gamma:=$gamma -p exposure_time:=$exposure_time \
-p action_trigger_mode:=true \
-p action_device_key:=$action_device_key -p action_group_key:=$action_group_key \
-p action_group_mask:=$action_group_mask -p action_lead_time:=$action_lead_time \
-p ptp_domain:=$ptp_domain"

# NOTE on node naming: the stock node name is hardcoded ("arena_camera_node"),
# so all six would share the same /<node>/trigger_image service and collide.
# We remap each node name with -r __node:=arenacamN. The absolute /trigger_all
# service is created only on the master, so it never collides.

# Camera 1  -- ACTION MASTER (fires /trigger_all; broadcasts the action command)
serial="254300057"
gnome-terminal --tab --title=cam1 -- bash -c "ros2 run arena_camera_node_trigger start --ros-args -r __node:=arenacam1 $common_args -p serial:=$serial -p topic:=/arenacam1/images -p camera_name:=arenacam1 -p action_master:=true -p action_trigger_rate:=$action_trigger_rate; /bin/bash"
sleep 1

# Camera 2
serial="254300058"
gnome-terminal --tab --title=cam2 -- bash -c "ros2 run arena_camera_node_trigger start --ros-args -r __node:=arenacam2 $common_args -p serial:=$serial -p topic:=/arenacam2/images -p camera_name:=arenacam2; /bin/bash"
sleep 1

# Camera 3
serial="254300053"
gnome-terminal --tab --title=cam3 -- bash -c "ros2 run arena_camera_node_trigger start --ros-args -r __node:=arenacam3 $common_args -p serial:=$serial -p topic:=/arenacam3/images -p camera_name:=arenacam3; /bin/bash"
sleep 1

# Camera 4
serial="254300056"
gnome-terminal --tab --title=cam4 -- bash -c "ros2 run arena_camera_node_trigger start --ros-args -r __node:=arenacam4 $common_args -p serial:=$serial -p topic:=/arenacam4/images -p camera_name:=arenacam4; /bin/bash"
sleep 1

# Camera 5
serial="254300055"
gnome-terminal --tab --title=cam5 -- bash -c "ros2 run arena_camera_node_trigger start --ros-args -r __node:=arenacam5 $common_args -p serial:=$serial -p topic:=/arenacam5/images -p camera_name:=arenacam5; /bin/bash"
sleep 1

# Camera 6
serial="254300054"
gnome-terminal --tab --title=cam6 -- bash -c "ros2 run arena_camera_node_trigger start --ros-args -r __node:=arenacam6 $common_args -p serial:=$serial -p topic:=/arenacam6/images -p camera_name:=arenacam6; /bin/bash"
