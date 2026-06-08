#!/bin/bash
if [ $# -eq 0 ] ; then
    echo "Usage: {private_workspace}"
    exit
fi

#workspace=/media/xusp/share_e/Develop/develop/agan_robot/slam-ws-ros2/src/ieskf-lio/
workspace=$1

echo "workspace: " $1

mv $workspace/rviz/agan_fastlio2.rviz $workspace/rviz/agan_ieskf_slam.rviz
sed -i 's/fastlio2/ieskf_slam/g' $workspace/package.xml
sed -i 's/fastlio2/ieskf_slam/g' $workspace/CMakeLists.txt
sed -i 's/fastlio2/ieskf_slam/g' $workspace/launch/agan_launch.py
sed -i 's/fastlio2/ieskf_slam/g' $workspace/rviz/agan_ieskf_slam.rviz

