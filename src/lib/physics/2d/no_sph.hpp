#pragma once

#include "fluid_solver.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

class NoSph : public FluidSolver {
public:
    virtual void computeTotalForce(Scene& scene, TimeStep dt);
};

}

} // d2

} // GooBalls

