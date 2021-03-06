#pragma once

#include "abstract_sph.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

/// This class wraps another Fluid solver and adds velocity correction based on particle positions
///
/// See "Volume Preserving Viscoelastic Fluids With Large Deformations using Position-based velocity Corrections" by Takahashi et al.
class ViscoElastic : public AbstractSph {
public:
    ViscoElastic();
    virtual void advance(Scene& scene, TimeStep dt) override;
    virtual void computeTotalForce(Scene& scene, TimeStep dt) override;
    virtual void initFluid(Scene& scene) override;
    virtual bool considerBoundary(bool consider);
    virtual bool considerBoundary() const;
    /// Set the fluid solver responsible for the core physics
    void base(std::unique_ptr<FluidSolver>&& b);
protected:
    void updateVelocityCorrectionCoefficients(Scene& scene, TimeStep dt);
    void mergeConnections(Scene& scene, Float slack = 1.0);
    void splitConnections(Scene& scene);
    void prepareFluidPositionCorrection(Scene& scene);
    void prepareBoundaryPositionCorrection(Scene& scene);
    void computeFluidPositionCorrection(const Scene& scene);
    void computeBoundaryPositionCorrection(const Scene& scene);
private:
    /// This is the main fluid solver who's results we adjust to make the fluid visco elastic
    std::unique_ptr<FluidSolver> m_solver;
    Coordinates2d m_fluid_dx;
    Coordinates2d m_boundary_dx;
};

} // Physics

} // d2

} // GooBalls
