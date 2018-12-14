#include "neighborhood_spatial_hashing.hpp"
#include "generic/eigen.hpp"

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <Eigen/Core>
#include <boost/log/trivial.hpp>


namespace GooBalls {

namespace Spatial {

NeighborhoodSpatialHashing::NeighborhoodSpatialHashing(){
    // empty
}

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

void NeighborhoodSpatialHashing::populateGrid(const GridCoordinates& targetGrid){
    auto& grid = m_grid;
    std::for_each(grid.begin(), grid.end(), [](auto& e){e.second.clear();});
    // fill grid
    for(int i = 0; i < targetGrid.rows(); ++i){
        grid[GridCoord(targetGrid(i,0), targetGrid(i,1))].push_back(i);
    }
}

inline void addCell(const std::vector<int>& cell, const Coordinates2d& target, const TranslationVector iv, const Float h2, std::vector<int>& indexes){
    for(int jj : cell){
        // we now have the coordinates of all candidates
        // compute squared distance and find good ones
        Float d2 = (target.row(jj) - iv).squaredNorm();
        if(d2 <= h2){
            indexes.push_back(jj);
        }
    }
}

void NeighborhoodSpatialHashing::computeInRange(
    const Coordinates2d& query, const Coordinates2d& target, 
    const GridCoordinates& queryGrid, const GridCoordinates& targetGrid, 
    Float h) {
    // reset output member members
    resetIndexes(query.rows());
    populateGrid(targetGrid);
    // guarantee: the indices in the per grid lists are sorted in ascending order (we get that for free)
    // for each point, find all neighbors in the ball with radius h
    const Float h2 = h*h;
    const auto& grid = m_grid;

    //#pragma omp parallel for schedule(static)
    for(int i = 0; i < query.rows(); ++i){
        auto& indexes = m_indexes[i];
        int ix = queryGrid(i, 0);
        int ixs = ix-1;
        int ixe = ix+1;
        int iy = queryGrid(i, 1);
        int iys = iy-1;
        int iye = iy+1;
        TranslationVector iv = query.row(i);
        for(int iix = ixs; iix <= ixe; iix++){
            for(int iiy = iys; iiy <= iye; iiy++){
                const auto coord = GridCoord(iix, iiy);
                const auto cell = grid.find(coord);
                if(cell == grid.end()){
                    continue;
                }
                addCell(cell->second, target, iv, h2, indexes);
            }
        }
    }

}

} // Spatial
} // GooBalls
