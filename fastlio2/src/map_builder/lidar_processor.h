#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sophus/se3.hpp>
#include "voxel_map.h"

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void initCloudMap(CloudType::Ptr init_cloud_world);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

private:
    Config   m_config;
    VoxelMap m_voxel_map;

    CloudType::Ptr m_map_cloud;

    pcl::KdTreeFLANN<PointType> m_kdtree;

    std::shared_ptr<IESKF> m_kf;
    CloudType::Ptr m_cloud_lidar;
    CloudType::Ptr m_cloud_down_lidar;

    //m x 4
    Eigen::MatrixXd   m_normal_matrix;
    Eigen::VectorXd   m_residuals;
    std::vector<char> m_point_selected_flag;
    std::vector<char> m_point_add_flag;

    CloudType::Ptr    m_cloud_world;

    pcl::VoxelGrid<PointType> m_scan_filter;
    pcl::VoxelGrid<PointType> m_map_filter;
};
