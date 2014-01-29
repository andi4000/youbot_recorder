#!/bin/bash

echo hurray
cd /home/ariandy/tmp/ros/;
rosbag record /cmd_vel /youbotStalker/object_tracking/object_detected /youbotStalker/object_tracking/cam_x_pos /youbotStalker/object_tracking/cam_y_pos /youbotStalker/object_tracking/distance & rosrun youbot_recorder record && fg
