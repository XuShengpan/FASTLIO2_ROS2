#include "local_map.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

LocalMap::LocalMap()
{
}

LocalMap::~LocalMap()
{
}

void LocalMap::init(pcl::PointCloud<PointType>::Ptr cloud_world)
{
    pcl::VoxelGrid<PointType> vg;
    pcl::PointCloud<PointType> out;
    vg.setLeafSize(0.2, 0.2, 0.2);
    vg.setInputCloud(cloud_world);
    vg.filter(out);

    m_map_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    *m_map_cloud = out;

    m_initialized = true;
}

void LocalMap::update(const Eigen::Vector3d& pos, pcl::PointCloud<PointType>::Ptr scan_world, const std::vector<int>& ptids)
{
    if (m_map_cloud == nullptr) {
        init(scan_world);
        return;
    }
    std::cout<<"scan size: "<<scan_world->size()<<std::endl;

    for (int ptid : ptids) {
        const auto& pt = scan_world->points[ptid];
        m_map_cloud->push_back(pt);
    }

    if (m_map_cloud->size() < m_config.map_cloud_size) {
        return;
    }

    int left = 0, right = m_map_cloud->size() - 1;
    const float map_half_length = m_config.map_cube_length;
    while (left < right) {
        while (left < right &&
            (abs(m_map_cloud->points[right].x - pos.x()) >   map_half_length
            || abs(m_map_cloud->points[right].y - pos.y()) > map_half_length
            || abs(m_map_cloud->points[right].z - pos.z()) > map_half_length))  right--;
        while (left < right
            && abs(m_map_cloud->points[left].x - pos.x()) < map_half_length
            && abs(m_map_cloud->points[left].y - pos.y()) < map_half_length
            && abs(m_map_cloud->points[left].z - pos.z()) < map_half_length)  left++;
        std::swap(m_map_cloud->points[left], m_map_cloud->points[right]);
    }
    if (right+1 < m_map_cloud->size()) {
        std::cout<<"LocalMap trimed: "<<m_map_cloud->size()<<" -> " << right+1<<std::endl;
        m_map_cloud->resize(right + 1);
    }

    std::cout<<"map size: "<<m_map_cloud->size()<<std::endl;
}

