#!/bin/bash
# If this fails, just manually run these commands

world_path="/usr/share/gazebo-7/worlds/"
launch_path="~/catkin_ws/src/ur_gazebo/launch/"
model_path="~/.gazebo/models/"
urdf_path="~/catkin_ws/src/ur_description/urdf/"
upload_path="~/catkin_ws/src/ur_description/launch/"

/bin/cp gzlaunch/* ~/catkin_ws/src/ur_gazebo/launch/ && echo "Copied launch files"
/bin/cp upload_launch/* ~/catkin_ws/src/ur_description/launch/ && echo "Copied upload launch files"
(/bin/cp worlds/* /usr/share/gazebo-7/worlds/ && echo "Copied world files") || echo "PLEASE RUN THIS AS ROOT"
/bin/cp -r models/* ~/.gazebo/models/ && echo "Copied model files"
/bin/cp urdf/* ~/catkin_ws/src/ur_description/urdf/ && echo "Copied URDF files" 
/bin/cp scripts/* ~/catkin_ws/src/ur_gazebo/scripts/ && echo "Copied Scripts"
echo "DONE"