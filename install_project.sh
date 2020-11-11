#!/bin/bash
# If this fails, just manually run these commands

world_path="/usr/share/gazebo-7/worlds/"
launch_path="~/catkin_ws/src/ur_gazebo/launch/"
model_path="~/.gazebo/models/"
urdf_path="~/catkin_ws/src/ur_description/urdf/"
upload_path="~/catkin_ws/src/ur_description/launch/"

cp gzlaunch/* launch_path && echo "Copied launch files"
cp upload_launch/* upload_path && echo "Copied upload launch files"
(cp worlds/* world_path && echo "Copied world files") || echo "PLEASE RUN THIS AS ROOT"
cp -r models/* ~/.gazebo/models/ && echo "Copied model files"
cp urdf/* ~/catkin_ws/src/ur_description/launch/ && echo "Copied URDF files" 
echo "DONE"