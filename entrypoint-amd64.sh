#!/bin/bash

nvidia-persistenced --verbose &

source /opt/ros/humble/setup.bash
source /opt/workspace/install/setup.bash
ros2 run zed_gst zed_cam
