#ifndef XSP_LIDAR_POINT_TYPE_H
#define XSP_LIDAR_POINT_TYPE_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

struct EIGEN_ALIGN16 PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)


#endif

