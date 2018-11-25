#pragma once

#include "types.hpp"
#include "fluid.hpp"


namespace GooBalls{
namespace d2 {
namespace Physics {

/**
 * Describes a floor, two walls and a ceiling.
 *
 * A NaN means: don't use this element
 * */
struct Room {
    CoordinatePrecision floor = -0.4;
    CoordinatePrecision left_wall = -1;
    CoordinatePrecision right_wall = 1;
    CoordinatePrecision ceiling = 10000;
    void restrictFluid(Fluid& fluid) const;
};

} // Physics
} // d2
} // GooBalls
