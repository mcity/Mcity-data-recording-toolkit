#!/bin/bash

# Fire ONE synchronized frame from all six LUCID cameras.
#
# Calls the action master's /trigger_all service. The master latches its PTP
# (RTK GPS) time, schedules an execute time, and broadcasts a GigE Vision action
# command that every camera receives -> one simultaneous frame per camera on
# each /arenacamN/images topic.
#
# Prereq: launch with ./lucid-drivers-action-trigger-mode.sh first, and wait for
# PTP to lock (the master refuses to fire until its PtpStatus == Slave).

ros2 service call /trigger_all std_srvs/srv/Trigger
