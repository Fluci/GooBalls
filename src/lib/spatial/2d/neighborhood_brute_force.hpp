#pragma once

#include "neighborhood.hpp"

namespace GooBalls {

namespace Spatial {

/**
 *  Reference implementation
 * */
class NeighborhoodBruteForce: public Neighborhood {
public:
    virtual void inRange(const Coordinates2d& points, CoordinatePrecision h);
    virtual void inRange(const Coordinates2d& query, const Coordinates2d& target_points, CoordinatePrecision h);
};


} // Spatial

} // GooBalls
