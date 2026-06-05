#include "voxel_map.h"
#include <unordered_set>

void VoxelMap::add_point(const PointType& point)
{
    CellPos pos = get_cell_pos(point.x, point.y, point.z);

    auto [it, inserted] = m_pos_index_map.try_emplace(pos, 0);

    CellIndexType index;
    if (inserted) {
        if (!m_free_indices.empty()) {
            index = m_free_indices.back();
            m_free_indices.pop_back();
            m_cells[index].clear();
        } else {
            index = m_cells.size();
            m_cells.emplace_back();
        }
        it->second = index; // 更新刚才插入的默认值
    } else {
        index = it->second;
    }

    m_cells[index].push_back(point);
}

void VoxelMap::add_points(const PointVec& points)
{
    for (const auto& point : points) {
        add_point(point);
    }
}

void VoxelMap::remove_far_voxels(const CellPos& center, int max_distance)
{
    for (auto it = m_pos_index_map.begin(); it != m_pos_index_map.end(); ) {
        const auto& pos = it->first;
        const auto& index = it->second;

        const int dx = pos.x - center.x;
        const int dy = pos.y - center.y;
        const int dz = pos.z - center.z;
        if (dx < -max_distance || dy < -max_distance || dz < -max_distance
            || dx > max_distance || dy > max_distance || dz > max_distance) {
            m_cells[index].clear();
            m_free_indices.push_back(index);
            it = m_pos_index_map.erase(it);
        } else {
            ++it;
        }
    }
}

void VoxelMap::remove_far_voxels(const Eigen::Vector3d& center)
{
    return remove_far_voxels(get_cell_pos(center[0], center[1], center[2]), m_max_distance);
}

void VoxelMap::gather_local_map(const PointVec& points_in, PointVec& points_out) const
{
    std::vector<CellIndexType> hit_indices;
    hit_indices.resize(points_in.size(), 0);

    const size_t size = points_in.size();
#pragma omp parallel for schedule(dynamic, 100)
    for (size_t i = 0; i < size; ++i) {
        const auto& point = points_in[i];
        CellPos pos = get_cell_pos(point.x, point.y, point.z);
        const auto it = m_pos_index_map.find(pos);
        if (it != m_pos_index_map.end()) {
            hit_indices[i] = it->second;
        }
    }

    // 排序并去重
    std::sort(hit_indices.begin(), hit_indices.end());
    hit_indices.erase(std::unique(hit_indices.begin(), hit_indices.end()), hit_indices.end());

    if (hit_indices.empty()) {
        points_out.clear();
        return;
    }

    size_t total_points = 0;
    for (CellIndexType idx : hit_indices) {
        total_points += m_cells[idx].size();
    }

    points_out.resize(total_points);

    size_t offset = 0;
    for (CellIndexType idx : hit_indices) {
        const auto& cell_points = m_cells[idx];
        if (!cell_points.empty()) {
            std::copy(cell_points.begin(), cell_points.end(), points_out.begin() + offset);
            offset += cell_points.size();
        }
    }
}
