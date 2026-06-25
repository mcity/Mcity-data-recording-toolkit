# Mcity-data-recording-toolkit
A collection of ROS2 drivers and recorder scripts to make data capture easier.

Our data captures usually include RTK positioning, lidar, and six cameras.

Hardware:

- OXTS AV200 RTK

- RoboSense lidars (either four M1 directional solid-state lidars, or one rotating Ruby+ lidar)

- Six Lucid cameras (either TRI023S-CC, or TRT023S-CC)

# Drivers
Main ones are "autoexposure" and "static". Stick with autoexposure unless you have a good reason; the new target luminance and gamma settings mostly resolve the discoloring issues with stock autoexposure settings. If you need trigger mode, you can set that flag to true in the config area.

There is also an optional, additional rate-limiter driver which re-publishes the camera topics at a reduced framerate.

## Synchronized trigger mode (PTP + action commands)
`lucid-drivers-sync-trigger.sh` launches all six cameras with `trigger_mode=true`, which now means **PTP-synchronized capture via scheduled GigE Vision action commands** (every camera exposes at the same instant in the OXTS RTK GPS timebase). One camera (cam1) is the action master.

- Launch: `./lucid-drivers-sync-trigger.sh`
- Wait for PTP to lock (master logs `PTP locked … 'Slave'`).
- Fire one synchronized shot: `./trigger-all.sh`
- Continuous: `./trigger-all-rate.sh 5` (≈5 Hz shell loop), or set `action_trigger_rate` in the launcher for a hardware-precise rate.

Camera topics are unchanged (`/arenacamN/images`), so the existing recorders work as-is. Timestamps are **RTK GPS time** (not UTC/Unix) — apply the GPS↔UTC offset downstream. Full details and the on-vehicle verification checklist: `arena_camera_node/SYNC_TRIGGERING.md`.

This **supersedes** `lucid-drivers-autoexposure-trigger-mode.sh`: with the updated driver, `trigger_mode=true` performs action-command sync (not software trigger), so that older script would configure the cameras but never fire.

# Recorders
Main one is `record-rtklidarcam.sh`. If you are using the rate-limiter and only want to record the rate-limited topics, you can use `record-rtklidarcam-rl.sh`
