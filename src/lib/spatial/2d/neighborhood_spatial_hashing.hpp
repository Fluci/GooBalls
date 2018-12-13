#pragma once

#include "neighborhood.hpp"
#include <unordered_map>


namespace std {
template<>
class hash<std::pair<int, int>> {
    public:
    size_t operator()(const std::pair<int, int> &p) const {
        size_t h1 = std::hash<int>()(p.first);
        size_t h2 = std::hash<int>()(p.second);
        return h1 ^ (h2 << 1);
    }
};
}


namespace GooBalls {

namespace Spatial {


/**
 * Proper interface: stores the results and provides access via accessors
 * */
class NeighborhoodSpatialHashing: public Neighborhood {
    typedef Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor> GridCoordinates;
    typedef std::pair<int, int> GridCoord;
    void computeInRange(const Coordinates2d& query, const Coordinates2d& target, const GridCoordinates& queryGrid, const GridCoordinates& targetGrid, Float h);
public:
    NeighborhoodSpatialHashing();
    virtual void inRange(const Coordinates2d& points, Float h);
    virtual void inRange(const Coordinates2d& query, const Coordinates2d& target_points, Float h);
private:
    // we keep most datastructures as members to avoid memory allocations
    std::unordered_map<GridCoord, std::vector<Index>> m_grid;
    void populateGrid(const GridCoordinates& target);
};

} // Spatial

} // GooBalls
