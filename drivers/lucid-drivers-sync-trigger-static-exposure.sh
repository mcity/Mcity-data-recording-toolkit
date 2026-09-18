#!/bin/bash

# ROS2 Camera drivers for LUCID cameras - PTP + Scheduled Action Command sync,
# with a FIXED (static) exposure and FIXED gain.
#
# Same synchronized capture as lucid-drivers-sync-trigger.sh (PTP + scheduled
# GigE Vision action commands, cam1 is the action master), except the cameras
# run open-loop: every frame in the run gets the same exposure and the same
# gain, so relative brightness across frames and across the six cameras is
# meaningful and the whole run can be lifted in post with one consistent
# adjustment.
#
# Deliberately exposed DARK. Under-exposing protects the highlights (a blown
# sky/headlight is unrecoverable; a dark frame is not) and is recovered in post.
# See "Tuning" below.
#
# Usage:
#   ./lucid-drivers-sync-trigger-static-exposure.sh                # config below
#   ./lucid-drivers-sync-trigger-static-exposure.sh 800            # exposure us
#   ./lucid-drivers-sync-trigger-static-exposure.sh 800 6.0        # exposure, gain dB
#
# The optional args exist because exposure is the one value you actually have to
# tune on the vehicle -- relaunch with a new number instead of editing the file.
#
# Picking the value on the vehicle (takes a minute, do it in the brightest
# light you expect to drive in):
#   1. launch this script, wait for PTP lock / frames flowing
#   2. in another terminal:  python3 check-exposure.py
#      it prints per-camera mean / highlight-clipping / shadow stats live
#   3. want:  clipped (>=250) well under ~0.5%  and  mean somewhere ~40-100
#      too bright / clipping -> relaunch with a lower number
#      pitch black (mean < ~15) -> raise it (gain only after exposure hits a
#      ceiling, see below)
#   Then recover brightness after the fact with ../postprocess/expose-in-post.py.
#
# Every camera logs one line "exposure state (STATIC): ExposureAuto=Off ...
# GainAuto=Off ..." right after it starts streaming. If that line says anything
# other than Off/Off, the run is NOT static -- stop and fix before recording.
#
# NOTE on target_brightness: this script never passes it. The driver now keeps
# exposure_time when both are given (and ignores target_brightness with a
# warning), but there is no reason to pass it in a static launch.
# lucid-drivers-sync-trigger.sh (target_brightness=70, no exposure_time) is the
# autoexposure launcher; this one is static.
#
# Documentation: arena_camera_node/SYNC_TRIGGERING.md  (section "Static exposure")

##### CONFIG #####
# bayer_bggr8 matches a 180-deg flip (reverse_x + reverse_y) of the RGGB sensor.
# With NO flip, use bayer_rggb8. See SYNC_TRIGGERING.md (Bayer caveat).
pixelformat="bayer_bggr8"
#pixelformat="bayer_bggr16"  # 12-bit in a 16-bit container: far more headroom
                             # for lifting a dark frame in post, but 2x the
                             # bandwidth and bag size. Consider it if 8-bit
                             # shadows come back too quantized/noisy.

# --- Static exposure ---------------------------------------------------------
# Exposure time in MICROSECONDS. Must be a FLOAT (4000.0, not 4000) -- the driver
# declares exposure_time as a double and rclcpp rejects an integer override.
#
# Starting point is intentionally LOW -- in direct sun a 2.3 MP Triton at 0 dB
# behind an f/2-ish lens saturates around ~300-600 us, so 500 us should sit
# slightly dark in full sun and clearly dark in overcast/shade. That is the
# intent: dark frames lift cleanly in post, blown highlights never come back.
# Tuning ladder:
#   too bright / clipping -> lower exposure_time
#   too dark              -> raise exposure_time first (cheapest, no added noise)
#   still dark            -> then raise gain, in small steps; gain is noise
#
# Two hard ceilings on exposure_time:
#   1. Trigger period. At action_trigger_rate=30 Hz the period is 33333 us, and
#      exposure must finish well inside it (frame readout + transmission also
#      live in that window). Stay under ~10000 us at 30 Hz.
#   2. Motion blur. At 30 mph (13.4 m/s) the vehicle moves ~13 mm per 1000 us.
#      1000-2000 us is fine; 8000 us smears at speed.
#
# The driver clamps this into the camera's legal ExposureTime range (and logs a
# warning) instead of dying, so a wrong number still launches.
exposure_time=500.0

# Fixed gain in dB. 0.0 = the sensor's clean floor, which is what you want when
# the plan is to recover exposure in post. Raise only after exposure_time has
# hit a ceiling above. Set gain="" to not pass it: the driver then still turns
# GainAuto Off and uses 0.0 dB whenever exposure_time is static, so the run
# stays static either way.
#
# Requires the static-exposure changes in ArenaCameraNode.cpp (rebuild
# arena_camera_node after pulling them).
gain=0.0
# -----------------------------------------------------------------------------

# Gamma < 1.0 lifts shadows before the 8-bit quantization, so a deliberately
# dark frame keeps more usable shadow detail to recover later. Note this makes
# the stored pixels NON-LINEAR: undo the gamma before any linear photometric
# work in post. Set gamma=1.0 if you need linear pixels straight out of the bag.
gamma=0.5

# NOTE: no target_brightness here -- on purpose. See the header note.

# Image flip in the camera (same as the ReverseX/ReverseY toggles in ArenaView).
# Set true to mirror. NOTE: flipping a Bayer image changes the effective Bayer
# pattern -- if colors look wrong after flipping, see SYNC_TRIGGERING.md.
reverse_x=true
reverse_y=true


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

# In-driver continuous rate (Hz) on the master. Set to 0.0 for one-shot only,
# then fire manually with ./trigger-all.sh.
action_trigger_rate=30.0

# Publisher QoS. For RECORDING, reliable avoids silently dropping frames when
# the six synchronized images burst into the recorder at once (best_effort
# discards on buffer overflow; reliable retransmits). keep_last + a few seconds
# of depth absorbs transient stalls without unbounded memory growth. Set
# qos_reliability=best_effort for low-latency live viewing if you are not
# recording.
qos_reliability=reliable
qos_history=keep_last
qos_history_depth=30

# --- Transmission staggering (helps HIGH-RATE recording; does NOT affect sync) ---
# Each camera delays transmitting its already-captured frame by
# (camera_index-1) * scftd_step, so the six synchronized frames arrive at the
# host spread out instead of all at once (which overflows DDS receive buffers at
# 30 Hz). This shapes TRANSMISSION only -- the PTP capture timestamp
# (header.stamp) and the ~us cross-camera sync are unchanged.
#
# Constraint: keep (5 * scftd_step) + one-frame transmission time BELOW the
# trigger period (1/rate). At 30 Hz the period is ~33 ms; with ~6 ms frames,
# ~4 ms steps put cam6 at 20 ms and finishing ~26 ms -- inside 33 ms.
scftd_step=4000000        # per-camera frame-transmission-delay step (ns); 0 = off
gev_scpd=0                # inter-packet delay (ns); 0 = off. Raise to throttle each camera's burst.

##################

# Optional CLI overrides: $1 = exposure (us), $2 = gain (dB).
if [ -n "$1" ]; then exposure_time="$1"; fi
if [ -n "$2" ]; then gain="$2"; fi

# The driver declares exposure_time and gain as doubles; rclcpp rejects an
# integer override and the node hangs on the rejected param. Force a decimal
# point so "1500" works on the command line.
case "$exposure_time" in *.*) ;; *) exposure_time="${exposure_time}.0" ;; esac

# gain="" means "don't pass it" -> the driver locks GainAuto Off at 0.0 dB anyway
# whenever exposure_time is given (static mode).
gain_arg=""
if [ -n "$gain" ]; then
    case "$gain" in *.*) ;; *) gain="${gain}.0" ;; esac
    gain_arg="-p gain:=$gain"
fi

echo "Static exposure: ${exposure_time} us, gain: ${gain:-0.0 (driver default)} dB, rate: ${action_trigger_rate} Hz"
echo "Check each camera's 'exposure state (STATIC)' log line reads ExposureAuto=Off GainAuto=Off."
echo "Live brightness/clipping per camera:  python3 $(dirname "$0")/check-exposure.py"
echo

# Common args for every camera (trigger_mode on -> synchronized action capture).
common_args="-p pixelformat:=$pixelformat -p gamma:=$gamma \
-p exposure_time:=$exposure_time $gain_arg \
-p reverse_x:=$reverse_x -p reverse_y:=$reverse_y \
-p trigger_mode:=true \
-p action_device_key:=$action_device_key -p action_group_key:=$action_group_key \
-p action_group_mask:=$action_group_mask -p action_lead_time:=$action_lead_time \
-p ptp_domain:=$ptp_domain \
-p qos_reliability:=$qos_reliability -p qos_history:=$qos_history -p qos_history_depth:=$qos_history_depth \
-p gev_scpd:=$gev_scpd"

# NOTE on node naming: the stock node name is hardcoded ("arena_camera_node"),
# so all six would share the same /<node>/trigger_image service and collide.
# We remap each node name with -r __node:=arenacamN. The absolute /trigger_all
# service is created only on the master, so it never collides.

# Camera 1  -- ACTION MASTER (fires /trigger_all; broadcasts the action command)
serial="254300057"
gnome-terminal --tab --title=cam1 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam1 $common_args -p serial:=$serial -p topic:=/arenacam1/images -p camera_name:=arenacam1 -p action_master:=true -p action_trigger_rate:=$action_trigger_rate -p gev_scftd:=$((0 * scftd_step)); /bin/bash"
sleep 1

# Camera 2
serial="254300058"
gnome-terminal --tab --title=cam2 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam2 $common_args -p serial:=$serial -p topic:=/arenacam2/images -p camera_name:=arenacam2 -p gev_scftd:=$((1 * scftd_step)); /bin/bash"
sleep 1

# Camera 3
serial="254300053"
gnome-terminal --tab --title=cam3 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam3 $common_args -p serial:=$serial -p topic:=/arenacam3/images -p camera_name:=arenacam3 -p gev_scftd:=$((2 * scftd_step)); /bin/bash"
sleep 1

# Camera 4
serial="254300056"
gnome-terminal --tab --title=cam4 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam4 $common_args -p serial:=$serial -p topic:=/arenacam4/images -p camera_name:=arenacam4 -p gev_scftd:=$((3 * scftd_step)); /bin/bash"
sleep 1

# Camera 5
serial="254300055"
gnome-terminal --tab --title=cam5 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam5 $common_args -p serial:=$serial -p topic:=/arenacam5/images -p camera_name:=arenacam5 -p gev_scftd:=$((4 * scftd_step)); /bin/bash"
sleep 1

# Camera 6
serial="254300054"
gnome-terminal --tab --title=cam6 -- bash -c "ros2 run arena_camera_node start --ros-args -r __node:=arenacam6 $common_args -p serial:=$serial -p topic:=/arenacam6/images -p camera_name:=arenacam6 -p gev_scftd:=$((5 * scftd_step)); /bin/bash"
