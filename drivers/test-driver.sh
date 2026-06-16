source ~/Downloads/arena_camera_ros2/ros2_ws/install/setup.bash

pixelformat="bayer_rggb8"
serial="254300055"
gamma=0.5
target_brightness=70

ros2 run arena_camera_node start --ros-args -p pixelformat:=$pixelformat -p gamma:=$gamma -p target_brightness:=$target_brightness -p serial:=$serial -p topic:=/arenacam0/images -p camera_name:=testcam
