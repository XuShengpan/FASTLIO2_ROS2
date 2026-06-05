#include "lidar_processor.h"
#include <algorithm>
#include <omp.h>

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_cloud_down_lidar.reset(new CloudType);

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });

    m_cloud_world = std::make_shared<CloudType>();
}

void LidarProcessor::initCloudMap(CloudType::Ptr init_cloud_world)
{
    m_local_map.init(init_cloud_world);
}

void LidarProcessor::process(SyncPackage &package)
{
    // m_kf->setLossFunction([&](State &s, SharedState &d)
    //                       { updateLossFunc(s, d); });
    // m_kf->setStopFunction([&](const V21D &delta) -> bool
    //                       { V3D rot_delta = delta.block<3, 1>(0, 0);
    //                         V3D t_delta = delta.block<3, 1>(3, 0);
    //                         return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
    //if (m_config.scan_resolution > 0.0)
    //{
    //    m_scan_filter.setInputCloud(package.lidar.cloud);
    //    m_scan_filter.filter(*m_cloud_down_lidar);
    //}
    //else
    //{
    //    pcl::copyPointCloud(*package.lidar.cloud, *m_cloud_down_lidar);
    //}

    m_cloud_down_lidar = package.lidar.cloud;
    const size_t last_size = m_point_selected_flag.size();
    const size_t size = m_cloud_down_lidar->points.size();

    if (size > last_size) {
        m_normal_matrix.resize(3, size);
        m_residuals.resize(size);
        m_point_selected_flag.resize(size);
        m_point_add_flag.resize(size);
    }

    m_cloud_world->resize(size);

    if (m_ptids_map.empty()) {
        m_kdtree.setInputCloud(m_local_map.get_map_cloud());
    } else {
        pcl::IndicesPtr indices = std::make_shared<pcl::Indices>(m_ptids_map);
        m_kdtree.setInputCloud(m_local_map.get_map_cloud(), indices);
    }

    m_kf->update();

    std::vector<int> ptids_add;
    for (int i=0; i<size; ++i) {
        if (m_point_add_flag[i]) {
            ptids_add.push_back(i);
        }
    }

    m_local_map.update(m_kf->x().t_wi, m_cloud_world, ptids_add);
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    const size_t size = m_cloud_down_lidar->size();
    const int last = m_config.near_search_num - 1;
    std::fill(m_point_selected_flag.begin(), m_point_selected_flag.end(), 0);
    std::fill(m_point_add_flag.begin(), m_point_add_flag.end(), 0);

    int effect_feat_num = 0;

    const auto map_cloud = m_local_map.get_map_cloud();
    const double map_resolution = m_local_map.get_config().map_resolution;
    const double min_sqr_dist_add = map_resolution * map_resolution;

    omp_set_num_threads(MP_PROC_NUM);

#pragma omp parallel
    {
        std::vector<float> point_sq_dist(m_config.near_search_num);
        std::vector<int>   ptids_near(m_config.near_search_num);

        PointVec points_near(m_config.near_search_num);

#pragma omp for schedule(dynamic) reduction(+:effect_feat_num)
    for (size_t i = 0; i < size; i++)
    {
        const PointType &point_body = m_cloud_down_lidar->points[i];
        PointType& point_world = m_cloud_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;

        m_kdtree.nearestKSearch(point_world, m_config.near_search_num, ptids_near, point_sq_dist);

        if (points_near.size() < m_config.near_search_num) {
            m_point_add_flag[i] = 1;
            continue;
        }

        if (point_sq_dist[0] > min_sqr_dist_add) {
            m_point_add_flag[i] = 1;
        }

        if (point_sq_dist[last] > 5.0)
            continue;

        for (int k=0;k<m_config.near_search_num;++k) {
            points_near[k] = map_cloud->points[ptids_near[k]];
        }

        Eigen::Vector4d pabcd;
        if (esti_plane(points_near, 0.1, pabcd))
        {
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            double s = 1.0 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9)
            {
                m_point_selected_flag[i] = 1;

                m_normal_matrix.col(i) = pabcd;
                m_residuals[i] = pd2;

                ++effect_feat_num;
            }
        }
    }
    }

    if (effect_feat_num < 10)
    {
        share_data.valid = false;
        std::cerr << "NO Effective Points!" << std::endl;
        return;
    }

    share_data.valid = true;
    share_data.H.setZero();
    share_data.b.setZero();

    const M3D r_wl = state.r_wi * state.r_il;

    Eigen::Vector3d norm_vec;
    Eigen::Matrix<double, 1, 3> B, C, D;
    for (size_t i = 0; i < size; i++) {
        if (!m_point_selected_flag[i])
            continue;

        Eigen::Matrix<double, 1, 12> J;
        J.setZero();
        const PointType &point_laser = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point_laser_vec(point_laser.x, point_laser.y, point_laser.z);
        norm_vec = m_normal_matrix.col(i);

        B = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * point_laser_vec + state.t_wi);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            C = -norm_vec.transpose() * r_wl * Sophus::SO3d::hat(point_laser_vec);
            D = norm_vec.transpose() * state.r_wi;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        share_data.H += J.transpose() * m_config.lidar_cov_inv * J;
        share_data.b += J.transpose() * m_config.lidar_cov_inv * m_residuals[i];
    }

}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}