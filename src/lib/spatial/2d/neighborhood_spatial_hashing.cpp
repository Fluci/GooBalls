#include "neighborhood_spatial_hashing.hpp"

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <Eigen/Core>

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


void NeighborhoodSpatialHashing::inRange(const Coordinates2d& points, CoordinatePrecision h){
    // compute
    /// We use a grid of size h and a hash table for near linear complexity
    GridCoordinates queryGridCoords = (points.array()/h).floor().cast<int>();
    computeInRange(points, points, queryGridCoords, queryGridCoords, h);
    assert(points.rows() == m_indexes.size());
    for(int i = 0; i < points.rows(); ++i){
        assert(!m_indexes[i].empty());
    }
}

void NeighborhoodSpatialHashing::inRange(const Coordinates2d& query, const Coordinates2d& target, CoordinatePrecision h){
    // compute
    /// We use a grid of size h and a hash table for near linear complexity
    GridCoordinates queryGridCoords = (query.array()/h).floor().cast<int>();
    GridCoordinates targetGridCoords = (target.array()/h).floor().cast<int>();
    computeInRange(query, target, queryGridCoords, targetGridCoords, h);
    assert(query.rows() == m_indexes.size());
}

void NeighborhoodSpatialHashing::computeInRange(
    const Coordinates2d& query, const Coordinates2d& target, 
    const GridCoordinates& queryGrid, const GridCoordinates& targetGrid, 
    CoordinatePrecision h) {
    // reset output member members
    resetIndexes(query.rows());
    std::unordered_map<GridCoord, std::vector<Index>> grid;
    // fill grid
    for(int i = 0; i < targetGrid.rows(); ++i){
        grid[GridCoord(targetGrid(i,0), targetGrid(i,1))].push_back(i);
    }   
    // guarantee: the indices in the per grid lists are sorted in ascending order (we get that for free)
    // for each point, find all neighbors in the ball with radius h
    CoordinatePrecision h2 = h*h;
    std::vector<Index> candidates;
    for(int i = 0; i < query.rows(); ++i){
        candidates.clear();
        for(int x = -1; x <= 1; x++){
            for(int y = -1; y <= 1; y++){
                auto coord = GridCoord(queryGrid(i, 0) + x, queryGrid(i, 1) + y);
                auto& cell = grid[coord];
                candidates.insert(candidates.end(), cell.begin(), cell.end());
            }   
        }
        Coordinates2d neighbors(candidates.size(), 2); 
        // copy coordinates for eigen math
        for(size_t j = 0; j < candidates.size(); ++j){
            neighbors.row(j) = target.row(candidates[j]);
        }
        // we now have the coordinates of all candidates
        // compute squared distance and find good ones
        auto difference = (neighbors.rowwise() - query.row(i));
        auto squared = difference.array() * difference.array();
        Coordinates1d squaredDistance = squared.rowwise().sum().eval();
        for(int j = 0; j < squaredDistance.rows(); ++j){
            CoordinatePrecision d2 = squaredDistance[j];
            if(d2 <= h2){
                m_indexes[i].push_back(candidates[j]);
            }
        }

    }

}

} // Spatial
} // GooBalls
