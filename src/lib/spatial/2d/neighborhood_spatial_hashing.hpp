#pragma once

#include "neighborhood.hpp"

namespace GooBalls {

namespace Spatial {


/**
 * Proper interface: stores the results and provides access via accessors
 * */
class NeighborhoodSpatialHashing: public Neighborhood {
    typedef Eigen::Matrix<int, Eigen::Dynamic, 2, Eigen::RowMajor> GridCoordinates;
    typedef std::pair<int, int> GridCoord;
    void computeInRange(const Coordinates2d& query, const Coordinates2d& target, const GridCoordinates& queryGrid, const GridCoordinates& targetGrid, CoordinatePrecision h);
public:
    virtual void inRange(const Coordinates2d& points, CoordinatePrecision h);
    virtual void inRange(const Coordinates2d& query, const Coordinates2d& target_points, CoordinatePrecision h);
};

} // Spatial

} // GooBalls
