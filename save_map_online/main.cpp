#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <string>
#include <atomic>
#include <filesystem>
#include "voxel_map.h"
#include <yaml-cpp/yaml.h>

class MapSaveNode : public rclcpp::Node
{
public:
    MapSaveNode() : Node("map_save_online_node"), save_counter_(0)
    {
        loadParameters();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, 100,
            std::bind(&MapSaveNode::cloud_callback, this, std::placeholders::_1));

        std::string filePath = __FILE__;
        auto sourceDir = std::filesystem::path(filePath).parent_path();
        output_folder_ = sourceDir.string() + "/" + output_folder_;
        if (!std::filesystem::exists(output_folder_)) {
            std::filesystem::create_directories(output_folder_);
        }

        voxel_map_.set_resolution(map_resolution_);

        RCLCPP_INFO(this->get_logger(), "Lidar Saver Node started. Subscribed to: %s", lidar_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output folder: %s", output_folder_.c_str());
    }

    ~MapSaveNode() {
        std::string map_file = output_folder_ + "/map_cloud.pcd";
        auto cloud = voxel_map_.get_point_cloud();
        pcl::io::savePCDFileBinary(map_file, *cloud);

        std::cout<<"Map cloud saved: "<<map_file<<std::endl;
    }

private:

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        // 读取业务参数，并赋予默认值以防 YAML 缺失导致崩溃
        lidar_topic_     = config["lidar_topic"].as<std::string>("/lidar_points_undistorted");
        output_folder_   = config["output_folder"].as<std::string>("./saved_clouds");
        world_frame_id_  = config["world_frame_id"].as<std::string>("./world");
        map_resolution_  = config["map_resolution"].as<double>(0.15);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::string frame_id = msg->header.frame_id;
        rclcpp::Time target_time(msg->header.stamp);
        geometry_msgs::msg::TransformStamped transform_stamped;

        try {
            transform_stamped = tf_buffer_->lookupTransform(
                world_frame_id_, frame_id, target_time, tf2::durationFromSec(0.05));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            RCLCPP_INFO(this->get_logger(), "All frames: %s", tf_buffer_->allFramesAsString().c_str());

            return;
        }

        const auto& q = transform_stamped.transform.rotation;
        const auto& p = transform_stamped.transform.translation;

        Sophus::SE3f T(Eigen::Quaternionf(q.w, q.x, q.y, q.z), Eigen::Vector3f(p.x, p.y, p.z));

        // 2. 将 ROS 消息转换为 PCL 点云
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(*msg, *cloud);

        // 3. 提取变换矩阵并应用变换
        //Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped);
        pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, T.matrix());

        voxel_map_.add_points(transformed_cloud->points);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string lidar_topic_;
    std::string world_frame_id_;
    std::string output_folder_;
    std::atomic<uint64_t> save_counter_;

    VoxelMap voxel_map_;

    double   map_resolution_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapSaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
