#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <sophus/se3.hpp>
#include "commons.h"

struct LocalMapConfig
{
    float  map_cube_length{ 600 };
    float  map_resolution{ 0.4 };
    int    map_cloud_size{ 30000 };
};

class LocalMap {
public:
    LocalMap();
    ~LocalMap();

    void init(pcl::PointCloud<PointType>::Ptr cloud_world);
    void update(const Eigen::Vector3d& position, pcl::PointCloud<PointType>::Ptr scan_world, const std::vector<int>& ptids);

    const pcl::PointCloud<PointType>::Ptr get_map_cloud() const
    {
        return m_map_cloud;
    }

    void set_config(const LocalMapConfig& cf)
    {
        m_config = cf;
    }

    const LocalMapConfig& get_config() const
    {
        return m_config;
    }

    bool is_initialized() const {return m_initialized;}

private:
    pcl::PointCloud<PointType>::Ptr m_map_cloud;
    LocalMapConfig m_config;

    bool m_initialized{ false };
};



