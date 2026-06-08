#!/bin/bash
if [ $# -eq 0 ] ; then
    echo "Usage: {private_workspace}"
    exit
fi

#workspace=/media/xusp/share_e/Develop/develop/agan_robot/slam-ws-ros2/src/ieskf-lio/
cd $1
mv fastlio2 ieskf-lio

workspace=$1/ieskf-lio

echo "workspace: " $workspace

mv $workspace/rviz/agan_fastlio2.rviz $workspace/rviz/agan_ieskf_slam.rviz

sed -i 's/fastlio2/ieskf_slam/g' $workspace/package.xml
sed -i 's/fastlio2/ieskf_slam/g' $workspace/CMakeLists.txt
sed -i 's/fastlio2/ieskf_slam/g' $workspace/launch/agan_launch.py
sed -i 's/fastlio2/ieskf_slam/g' $workspace/rviz/agan_ieskf_slam.rviz

cd $workspace/src/map_builder
mv voxel_map.h local_map.h
mv voxel_map.cpp local_map.cpp

echo "current forlder: " `pwd`

sed -i 's/VoxelMap/LocalMap/g'  local_map.h
sed -i 's/VoxelMap/LocalMap/g'  local_map.cpp
sed -i 's/voxel_map/local_map/g'  local_map.cpp

sed -i 's/VoxelMap/LocalMap/g'  lidar_processor.h
sed -i 's/voxel_map/local_map/g'  lidar_processor.h
sed -i 's/voxel_map/local_map/g'  lidar_processor.cpp

sed -i 's/m_free_indices/m_buffer/g' local_map.h
sed -i 's/m_free_indices/m_buffer/g' local_map.cpp

sed -i 's/m_cells/m_data/g' local_map.h
sed -i 's/m_cells/m_data/g' local_map.cpp

sed -i 's/remove_far_voxels/update/g' local_map.h
sed -i 's/remove_far_voxels/update/g' local_map.cpp
sed -i 's/remove_far_voxels/update/g' lidar_processor.cpp
sed -i 's/remove_far_voxels/update/g' lidar_processor.cpp

sed -i 's/gather_local_map/get/g' local_map.h
sed -i 's/gather_local_map/get/g' local_map.cpp
sed -i 's/gather_local_map/get/g' lidar_processor.cpp
sed -i 's/gather_local_map/get/g' lidar_processor.cpp

sed -i 's/add_points/set_points/g' local_map.h
sed -i 's/add_points/set_points/g' local_map.cpp
sed -i 's/add_points/set_points/g' lidar_processor.cpp
sed -i 's/add_points/set_points/g' lidar_processor.cpp

sed -i 's/m_point_selected_flag/m_flag1/g' lidar_processor.h
sed -i 's/m_point_selected_flag/m_flag1/g' lidar_processor.cpp

sed -i 's/m_point_add_flag/m_flag2/g' lidar_processor.h
sed -i 's/m_point_add_flag/m_flag2/g' lidar_processor.cpp

sed -i 's/m_normal_matrix/m_data1/g' lidar_processor.h
sed -i 's/m_normal_matrix/m_data1/g' lidar_processor.cpp

sed -i 's/m_residuals/m_data2/g' lidar_processor.h
sed -i 's/m_residuals/m_data2/g' lidar_processor.cpp

sed -i 's/m_shared_data/m_data/g' ieskf.h
sed -i 's/m_shared_data/m_data/g' ieskf.cpp

sed -i 's/predict/forward/g' ieskf.h
sed -i 's/predict/forward/g' ieskf.cpp
sed -i 's/predict/forward/g' imu_processor.cpp

sed -i 's/r_wi/r/g' ieskf.h
sed -i 's/r_wi/r/g' ieskf.cpp
sed -i 's/r_wi/r/g' imu_processor.h
sed -i 's/r_wi/r/g' imu_processor.cpp
sed -i 's/r_wi/r/g' lidar_processor.h
sed -i 's/r_wi/r/g' lidar_processor.cpp
sed -i 's/r_wi/r/g' map_builder.h
sed -i 's/r_wi/r/g' map_builder.cpp

sed -i 's/t_wi/p/g' ieskf.h
sed -i 's/t_wi/p/g' ieskf.cpp
sed -i 's/t_wi/p/g' imu_processor.h
sed -i 's/t_wi/p/g' lidar_processor.h
sed -i 's/t_wi/p/g' imu_processor.cpp
sed -i 's/t_wi/p/g' lidar_processor.cpp
sed -i 's/t_wi/p/g' map_builder.h
sed -i 's/t_wi/p/g' map_builder.cpp

sed -i 's/m_last_acc/m_u/g' imu_processor.h
sed -i 's/m_last_acc/m_u/g' imu_processor.cpp

sed -i 's/m_last_gyro/m_v/g' imu_processor.h
sed -i 's/m_last_gyro/m_v/g' imu_processor.cpp

sed -i 's/IMUProcessor/UpdateProcessor/g' imu_processor.h
sed -i 's/IMUProcessor/UpdateProcessor/g' imu_processor.cpp
sed -i 's/IMUProcessor/UpdateProcessor/g' map_builder.h
sed -i 's/IMUProcessor/UpdateProcessor/g' map_builder.cpp

sed -i 's/LidarProcessor/CorrectProcessor/g' lidar_processor.h
sed -i 's/LidarProcessor/CorrectProcessor/g' lidar_processor.cpp
sed -i 's/LidarProcessor/CorrectProcessor/g' map_builder.h
sed -i 's/LidarProcessor/CorrectProcessor/g' map_builder.cpp

sed -i 's/lidar_processor/correct_processor/g' lidar_processor.cpp
sed -i 's/lidar_processor/correct_processor/g' map_builder.h
sed -i 's/lidar_processor/correct_processor/g' map_builder.cpp

sed -i 's/imu_processor/update_processor/g' imu_processor.cpp
sed -i 's/imu_processor/update_processor/g' map_builder.h
sed -i 's/imu_processor/update_processor/g' map_builder.cpp

mv lidar_processor.h correct_processor.h
mv lidar_processor.cpp correct_processor.cpp

mv imu_processor.h update_processor.h
mv imu_processor.cpp update_processor.cpp

cd ..
sed -i 's/imu_processor/update_processor/g' lio_node.cpp
sed -i 's/lidar_processor/correct_processor/g' lio_node.cpp
sed -i 's/r_wi/r/g' lio_node.cpp
sed -i 's/t_wi/p/g' lio_node.cpp

echo "Finished."


