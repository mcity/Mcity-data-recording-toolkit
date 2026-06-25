#!/bin/bash

# Continuously fire synchronized frames by calling /trigger_all in a shell loop.
#
# Usage: ./trigger-all-rate.sh [rate_hz]   (default 5 Hz)
#
# NOTE: this shell loop's spacing is approximate (subject to service-call and
# shell timing). Each individual shot is still fully PTP-synchronized ACROSS the
# six cameras. For a hardware-precise, evenly spaced rate, prefer the in-driver
# timer instead: launch the master with action_trigger_rate:=<hz> (see
# lucid-drivers-action-trigger-mode.sh) and do NOT run this script.
#
# Prereq: launch with ./lucid-drivers-action-trigger-mode.sh and wait for PTP lock.

rate_hz="${1:-5}"
period=$(awk "BEGIN { print 1.0 / $rate_hz }")

echo "Firing /trigger_all at ~${rate_hz} Hz (period ${period}s). Ctrl-C to stop."
while true; do
    ros2 service call /trigger_all std_srvs/srv/Trigger
    sleep "$period"
done
