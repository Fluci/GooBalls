#include "fluid_solver.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {


bool FluidSolver::considerBoundary(bool consider) {
    return false;
}

bool FluidSolver::considerBoundary() const {
    return false;
}


} // Physics

} // d2

} // GooBalls
