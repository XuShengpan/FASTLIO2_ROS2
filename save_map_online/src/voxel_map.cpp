#include "voxel_map.h"
#include <unordered_set>

static double one_per_n[4096];

static void init_one_per_n()
{
    for (int i=1; i<4097; ++i) {
        one_per_n[i] = 1.0 / double(i);
    }
}

void CellDataType::push(const PointType& point)
{
    ++num;
    if (num < 4097) {
        x += (point.x - x) * one_per_n[num];
        y += (point.y - y) * one_per_n[num];
        z += (point.z - z) * one_per_n[num];
        intensity += (point.intensity - intensity) * one_per_n[num];
    } else {
        double t = 1.0/double(num);
        x += (point.x - x) * t;
        y += (point.y - y) * t;
        z += (point.z - z) * t;
        intensity += (point.intensity - intensity) * t;
    }
}

VoxelMap::VoxelMap(float resolution)
    : m_resolution(resolution), m_inv_resolution(1.0f / resolution) {

    m_cells.reserve(4096);
    m_pos_index_map.reserve(4096);

    init_one_per_n();
}

void VoxelMap::add_point(const PointType& point)
{
    CellPos pos = get_cell_pos(point.x, point.y, point.z);

    auto [it, inserted] = m_pos_index_map.try_emplace(pos, 0);

    CellIndexType index;
    if (inserted) {
        index = m_cells.size();
        m_cells.emplace_back();
    } else {
        index = it->second;
    }

    m_cells[index].push(point);
}

void VoxelMap::add_points(const PointVec& points)
{
    for (const auto& point : points) {
        add_point(point);
    }
}

pcl::PointCloud<PointType>::Ptr VoxelMap::get_point_cloud() const
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud->reserve(m_cells.size());
    for (const auto& cell : m_cells) {
        PointType pt;
        pt.x = cell.x;
        pt.y = cell.y;
        pt.z = cell.z;
        pt.intensity = cell.intensity;
        pt.normal_x = cell.num;
        cloud->push_back(pt);
    }

    return cloud;
}
