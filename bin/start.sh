#!/bin/bash

echo hurray

cd ~;
DIR="youbot_stalker_measurements/data_`date +%Y_%m_%d`";
FILENAME="youbotStalker_`date +%Y-%m-%d_%H-%M`"
mkdir -p $DIR;
cd $DIR;

rosbag record -O $FILENAME.bag /cmd_vel /youbotStalker/object_tracking/object_detected /youbotStalker/object_tracking/cam_x_pos /youbotStalker/object_tracking/cam_y_pos /youbotStalker/object_tracking/distance /youbotStalker/gesture_processor/state /youbotStalker/gesture_processor/offset_linear_x /youbotStalker/gesture_processor/offset_linear_y & rosrun youbot_recorder record && fg
