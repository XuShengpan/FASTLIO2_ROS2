#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZINormal;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType> >;

struct CellDataType
{
    float  x{0};
    float  y{0};
    float  z{0};
    float  intensity{0};
    size_t num {0};

    inline void push(const PointType& point);
};

class VoxelMap {
public:
    using CellIndexType = size_t;

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

private:

    float m_resolution {1.0};
    float m_inv_resolution {1.0};

    // 核心数据结构
    std::unordered_map<CellPos, CellIndexType, CellPosHash> m_pos_index_map;
    std::vector<CellDataType>  m_cells;

public:
    explicit VoxelMap(float resolution = 1.0f);

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

    void set_resolution(float resolution)
    {
        m_resolution = resolution;
        m_inv_resolution = 1.0f / resolution;
    }

    void add_point(const PointType& point);

    void add_points(const PointVec& points);

    pcl::PointCloud<PointType>::Ptr get_point_cloud() const;
};
