#!/bin/bash

# ROS2 Camera drivers for LUCID cameras - PTP + Scheduled Action Command sync.
#
# Launches all six cameras with trigger_mode=true, which (in this driver) means
# synchronized capture via PTP + scheduled GigE Vision action commands. Exactly
# ONE camera (cam1) is the action master: it owns the /trigger_all service and
# broadcasts the scheduled action command that fires every camera at the same
# instant in the OXTS RTK GPS timebase.
#
# Fire a synchronized shot with ./trigger-all.sh (or a steady rate with
# ./trigger-all-rate.sh / the action_trigger_rate config below).
#
# NOTE: this SUPERSEDES lucid-drivers-autoexposure-trigger-mode.sh. With the
# updated driver, trigger_mode=true performs action-command sync (not software
# trigger), so that older script would configure the cameras but never fire.
# Documentation: arena_camera_node/SYNC_TRIGGERING.md

##### CONFIG #####
pixelformat="bayer_rggb8" # Default, faster
#pixelformat="rgb8"
#pixelformat="bayer_rggb16"

gamma=0.5

# Fixed exposure (microseconds) is recommended for deterministic synchronized
# capture under triggering. [verify] tune this for the rig/scene on the vehicle.
# NOTE: must be a FLOAT (e.g. 4000.0). The driver declares exposure_time as a
# double, so rclcpp rejects an integer override like 4000.
exposure_time=4000.0

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

# In-driver continuous rate (Hz) on the master. Defaults to 10 Hz so a fresh
# launch self-fires (needed for continuous recording). Set to 0.0 for one-shot
# only, then fire manually with ./trigger-all.sh.
action_trigger_rate=10.0

# Publisher QoS. For RECORDING, reliable avoids silently dropping frames when
# the six synchronized images burst into the recorder at once (best_effort
# discards on buffer overflow; reliable retransmits). keep_last + a few seconds
# of depth absorbs transient stalls without unbounded memory growth. Set
# qos_reliability=best_effort for low-latency live viewing if you are not
# recording.
qos_reliability=reliable
qos_history=keep_last
qos_history_depth=30

##################

# Common args for every camera (trigger_mode on -> synchronized action capture).
common_args="-p pixelformat:=$pixelformat -p gamma:=$gamma -p exposure_time:=$exposure_time \
-p trigger_mode:=true \
-p action_device_key:=$action_device_key -p action_group_key:=$action_group_key \
-p action_group_mask:=$action_group_mask -p action_lead_time:=$action_lead_time \
-p ptp_domain:=$ptp_domain \
-p qos_reliability:=$qos_reliability -p qos_history:=$qos_history -p qos_history_depth:=$qos_history_depth"

# NOTE on node naming: the stock node name is hardcoded ("arena_camera_node"),
# so all six would share the same /<node>/trigger_image service and collide.
# We remap each node name with -r __node:=arenacamN. The absolute /trigger_all
# service is created only on the master, so it never collides.

# Camera 1  -- ACTION MASTER (fires /trigger_all; broadcasts the action command)
serial="254300057"
gnome-terminal --tab --title=cam1 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam1 $common_args -p serial:=$serial -p topic:=/arenacam1/images -p camera_name:=arenacam1 -p action_master:=true -p action_trigger_rate:=$action_trigger_rate; /bin/bash"
sleep 1

# Camera 2
serial="254300058"
gnome-terminal --tab --title=cam2 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam2 $common_args -p serial:=$serial -p topic:=/arenacam2/images -p camera_name:=arenacam2; /bin/bash"
sleep 1

# Camera 3
serial="254300053"
gnome-terminal --tab --title=cam3 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam3 $common_args -p serial:=$serial -p topic:=/arenacam3/images -p camera_name:=arenacam3; /bin/bash"
sleep 1

# Camera 4
serial="254300056"
gnome-terminal --tab --title=cam4 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam4 $common_args -p serial:=$serial -p topic:=/arenacam4/images -p camera_name:=arenacam4; /bin/bash"
sleep 1

# Camera 5
serial="254300055"
gnome-terminal --tab --title=cam5 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam5 $common_args -p serial:=$serial -p topic:=/arenacam5/images -p camera_name:=arenacam5; /bin/bash"
sleep 1

# Camera 6
serial="254300054"
gnome-terminal --tab --title=cam6 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam6 $common_args -p serial:=$serial -p topic:=/arenacam6/images -p camera_name:=arenacam6; /bin/bash"
