#pragma once

#include "fluid_solver.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

/// This class wraps another Fluid solver and
class ViscoElastic : public FluidSolver {
public:
    ViscoElastic();
    virtual void advance(Scene& scene, TimeStep dt);
    virtual void computeTotalForce(Scene& scene, TimeStep dt);
private:
    /// This is the main fluid solver who's results we adjust to make the fluid visco elastic
    std::unique_ptr<FluidSolver> m_solver;
};

} // Physics

} // d2

} // GooBalls
