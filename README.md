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

# Recorders
Main one is `record-rtklidarcam.sh`. If you are using the rate-limiter and only want to record the rate-limited topics, you can use `record-rtklidarcam-rl.sh`
