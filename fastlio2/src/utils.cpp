#include "utils.h"
#include "lidar_point_type.h"
#include <pcl_conversions/pcl_conversions.h>

#include "map_builder/commons.h"

LidarFrame Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    const double start_time = getSec(msg->header);
    double offset_time_max = -1;

    CloudType::Ptr cloud = std::make_shared<CloudType>();
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;

            if (msg->points[i].offset_time > offset_time_max) {
                offset_time_max = msg->points[i].offset_time;
            }

            cloud->emplace_back(p);
        }
    }

    return  LidarFrame(start_time, start_time + offset_time_max, cloud);
}

LidarFrame Utils::hesai2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    int filter_num, double min_range, double max_range) {

    pcl::PointCloud<PointXYZIRT> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);

    const double start_time = getSec(msg->header);
    double end_time = -1;

    CloudType::Ptr cloud = std::make_shared<CloudType>();
    int point_num = raw_cloud.size();

    for (int i = 0; i < point_num; i += filter_num)
    {
        const auto& pt = raw_cloud.points[i];
        {
            float dist2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (dist2 < min_range * min_range || dist2 > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            p.curvature = (pt.timestamp - start_time) * 1000.0;
            if (pt.timestamp > end_time) {
                end_time = pt.timestamp;
            }

            cloud->emplace_back(p);
        }
    }

    return  LidarFrame(start_time, end_time, cloud);

}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
