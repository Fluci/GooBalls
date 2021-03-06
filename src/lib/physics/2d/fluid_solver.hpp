#pragma once

#include "types.hpp"
#include "scene.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

class FluidSolver {
public:
    virtual ~FluidSolver() = default;
    /**
    * Simulates a timestep of size `dt` on `scene`
    **/
    virtual void advance(Scene& scene, TimeStep dt) = 0;
    /**
    * Computes the total force for each particle. `dt` is not yet applied to it
    * The total force is stored in `scene.fluid`
    **/
    virtual void computeTotalForce(Scene& scene, TimeStep dt) = 0;

    /// Returns true, if it supports boundaries
    virtual bool considerBoundary(bool consider);
    virtual bool considerBoundary() const;

    virtual void initFluid(Scene& scene);
};


} // Physics

} // d2

} // GooBalls

