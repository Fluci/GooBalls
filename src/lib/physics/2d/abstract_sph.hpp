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
 * This class collects the most common algorithm-parts needed to implement an SPH.
 * */
class AbstractSph : public FluidSolver {
protected:
    Coordinates2d FGravity;
    Coordinates2d FPressure;
    Coordinates2d FViscosity;
    Coordinates2d FSurface;
    virtual void advance(Scene& scene, TimeStep dt);
    /// resizes matrices etc.
    void prepareFluid(Scene& scene) const;
    /// resizes matrices etc.
    void prepareBoundary(Scene& scene) const;
    /// computes rho_i += sum_j m_j W_ij, for i and j being fluid particles
    void addFluidDensity(Scene& scene, const Kernel& densityKernel) const;
    /// p := rho0 * K / gamma ((rho / rho0)^gamma - 1)
    void computeFluidPressure(Scene& scene) const;
    /// s.t. F = m * scene.gravity
    void computeGravityForce(const Scene& scene);
    /// As described in Mueller et al. 2003
    void computeStandardViscosityForce(const Scene& scene, const Kernel&);
    /// As described in Mueller et al. 2003
    void computeStandardPressureForce(const Scene& scene, const Kernel& pressureKernel);
    /// As described in Mueller et al. 2003
    void computeStandardSurfaceTensionForce(const Scene& scene, const Kernel& kernel, FloatPrecision color_relevant_normal_size);
public:
    virtual ~AbstractSph() = default;
};

} // Physics

} // d2

} // GooBalls

