#pragma once

#include "fluid_solver.hpp"
#include "kernel.hpp"
#include "spatial/2d/neighborhood.hpp"
#include <memory>

namespace GooBalls {

namespace d2 {

namespace Physics {

using namespace Spatial;

/**
 * Standard smooth particle hydrodynamics according to the Paper by Müller et al. from 2003
 * SSPH models a compressible fluid!
 * */
class SSPH : public FluidSolver {
private:
    std::unique_ptr<Neighborhood> m_neighborhood;
    std::unique_ptr<Neighborhood> m_boundary_neighborhood;
    std::unique_ptr<Kernel> m_kernel;
    bool m_consider_boundary = false;
    /// force from fluid to boundary particles
    Coordinates2d m_boundary_force;
public:
    SSPH();
    virtual ~SSPH() = default;
    virtual void computeTotalForce(Scene& scene, TimeStep dt);

};

} // Physics

} // d2

} // GooBalls

