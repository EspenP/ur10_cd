#!/bin/bash

world_path="/usr/share/gazebo-7/worlds/"
launch_path="~/catkin_ws/src/ur_gazebo/launch/"
model_path="~/.gazebo/models/"

cp -r launch/* launch_path && echo "Copied launch files"
(cp -r worlds/* world_path && echo "Copied world files") || echo "PLEASE RUN THIS AS ROOT"
cp -r models/* model_path && echo "Copied model files"
echo "DONE"