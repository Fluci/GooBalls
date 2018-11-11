#pragma once

#include "types.hpp"
#include <vector>

namespace GooBalls {

namespace Spatial {

/**
 * Interface
 * */
class Neighborhood {
public:
    typedef int Index;
protected:
    std::vector<std::vector<Index>> m_indexes;
    void resetIndexes(int size);
public:
    std::vector<std::vector<Index>>& indexes();
    const std::vector<std::vector<Index>>& indexes() const;
/// takes a list of points (1 row = 1 point) and a radius `h`
/// It computes for each point i a list of points within radius `h` (index i is not contained) and returns this list
/// the per point index lists are not sorted
/// the list of a point i contains the point i itself
    virtual void inRange(const Coordinates2d& points, CoordinatePrecision h) = 0;
/// Takes a list of query points a nd target points. For each query point, the index of the target points in radius `h` is returned.
    virtual void inRange(const Coordinates2d& query, const Coordinates2d& target_points, CoordinatePrecision h) = 0;
};

} // Spatial

} // GooBalls
