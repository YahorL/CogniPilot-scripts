#!/bin/bash

# Publish a fake odometry message
# This is useful for testing the navigation stack
# It publishes a message with a constant position and orientation
# and a constant velocity
# The message is published on the /cf1/odom topic
# The message is published at a rate of 10 Hz

ros2 topic pub /cf1/odom nav_msgs/msg/Odometry "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'odom'
child_frame_id: 'base_link'
pose:
  pose:
    position:
      x: 20.0
      y: 30.0
      z: 40.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]"