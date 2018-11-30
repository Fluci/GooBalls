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
    Float floor = -10000000;
    Float left_wall = -1000000;
    Float right_wall = 1000000;
    Float ceiling = 1000000;
    void restrictFluid(Fluid& fluid) const;
};

} // Physics
} // d2
} // GooBalls
