#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include <Eigen/Core>
#include "commons.h"

class VoxelMap {
public:
    using CellIndexType = size_t;
    using CellDataType = PointVec;

private:
    struct CellPos {
        int x, y, z;
        bool operator==(const CellPos& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellPosHash {
        size_t operator()(const CellPos& pos) const noexcept {
            constexpr size_t p1 = 73856093;
            constexpr size_t p2 = 19349669;
            constexpr size_t p3 = 83492791;

            return (static_cast<size_t>(pos.x) * p1) ^
                   (static_cast<size_t>(pos.y) * p2) ^
                   (static_cast<size_t>(pos.z) * p3);
        }
    };

    inline CellPos get_cell_pos(float x, float y, float z) const {
        return {
            static_cast<int>(std::floor(x * m_inv_resolution)),
            static_cast<int>(std::floor(y * m_inv_resolution)),
            static_cast<int>(std::floor(z * m_inv_resolution))
        };
    }

    void remove_far_voxels(const CellPos& pos, int max_distance);

private:

    float m_resolution {1.0};
    float m_inv_resolution {1.0};
    float m_cube_size {200.0};
    int   m_max_distance {100};

    // 核心数据结构
    std::unordered_map<CellPos, CellIndexType, CellPosHash> m_pos_index_map;
    std::vector<CellDataType>  m_cells;
    std::vector<CellIndexType> m_free_indices;

public:
    explicit VoxelMap(float resolution = 1.0f)
        : m_resolution(resolution), m_inv_resolution(1.0f / resolution) {
        m_cells.reserve(1024);
        m_pos_index_map.reserve(1024);
    }

    void set_cube_size(float cube_size) {
        m_cube_size = cube_size;
        m_max_distance = 0.5 * cube_size * m_inv_resolution;
    }

    //if not found, return -1
    inline int get_cell_index(float x, float y, float z) const {
        auto pos = get_cell_pos(x, y, z);
        auto it = m_pos_index_map.find(pos);
        if (it == m_pos_index_map.end()) {
            return  -1;
        } else {
            return static_cast<int>(it->second);
        }
    }

    void add_point(const PointType& point);

    void add_points(const PointVec& points);

    void remove_far_voxels(const Eigen::Vector3d& center);

    void gather_local_map(const PointVec& points_in, PointVec& points_out) const;
};
