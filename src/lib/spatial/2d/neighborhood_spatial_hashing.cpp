#include "neighborhood_spatial_hashing.hpp"
#include "generic/eigen.hpp"

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <Eigen/Core>


namespace GooBalls {

namespace Spatial {


void NeighborhoodSpatialHashing::inRange(const Coordinates2d& points, Float h){
    // compute
    /// We use a grid of size h and a hash table for near linear complexity
    GridCoordinates queryGridCoords = (points.array()/h).floor().cast<int>();
    computeInRange(points, points, queryGridCoords, queryGridCoords, h);
    assert(points.rows() == m_indexes.size());
    for(int i = 0; i < points.rows(); ++i){
        assert(!m_indexes[i].empty());
    }
}

void NeighborhoodSpatialHashing::inRange(const Coordinates2d& query, const Coordinates2d& target, Float h){
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
    Float h) {
    // reset output member members
    resetIndexes(query.rows());
    auto& grid = m_grid;
    grid.clear();
    // fill grid
    for(int i = 0; i < targetGrid.rows(); ++i){
        grid[GridCoord(targetGrid(i,0), targetGrid(i,1))].push_back(i);
    }   
    // guarantee: the indices in the per grid lists are sorted in ascending order (we get that for free)
    // for each point, find all neighbors in the ball with radius h
    Float h2 = h*h;
    auto& candidates = m_candidates;
    Coordinates2d& neighbors = m_neighbors; 
    Coordinates1d& squaredDistance = m_squaredDistance;

    for(int i = 0; i < query.rows(); ++i){
        candidates.clear();
        for(int x = -1; x <= 1; x++){
            for(int y = -1; y <= 1; y++){
                auto coord = GridCoord(queryGrid(i, 0) + x, queryGrid(i, 1) + y);
                auto& cell = grid[coord];
                candidates.insert(candidates.end(), cell.begin(), cell.end());
            }   
        }
        int N = candidates.size();
        minSize(neighbors, N);
        minSize(squaredDistance, N);

        // copy coordinates for eigen math
        for(size_t j = 0; j < N; ++j){
            neighbors.row(j) = target.row(candidates[j]);
        }
        // we now have the coordinates of all candidates
        // compute squared distance and find good ones
        auto difference = (neighbors.block(0,0,N,2).rowwise() - query.row(i));

        squaredDistance.block(0,0,N,1) = difference.rowwise().squaredNorm();
        for(int j = 0; j < N; ++j){
            Float d2 = squaredDistance[j];
            if(d2 <= h2){
                m_indexes[i].push_back(candidates[j]);
            }
        }

    }

}

} // Spatial
} // GooBalls
