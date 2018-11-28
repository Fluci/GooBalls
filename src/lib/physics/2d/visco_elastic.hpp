#pragma once

#include "fluid_solver.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

/// This class wraps another Fluid solver and adds velocity correction based on particle positions
class ViscoElastic : public FluidSolver {
public:
    ViscoElastic();
    virtual void advance(Scene& scene, TimeStep dt);
    virtual void computeTotalForce(Scene& scene, TimeStep dt);
    virtual bool considerBoundary(bool consider);
    virtual bool considerBoundary() const;
    /// Set the fluid solver responsible for the core physics
    void base(std::unique_ptr<FluidSolver>&& b);
private:
    /// This is the main fluid solver who's results we adjust to make the fluid visco elastic
    std::unique_ptr<FluidSolver> m_solver;
};

} // Physics

} // d2

} // GooBalls
