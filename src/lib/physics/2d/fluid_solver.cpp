#include "fluid_solver.hpp"

#include <boost/log/trivial.hpp>

namespace GooBalls {

namespace d2 {

namespace Physics {


bool FluidSolver::considerBoundary(bool) {
    return false;
}

bool FluidSolver::considerBoundary() const {
    return false;
}

void FluidSolver::initFluid(Scene&) {
    // empty
    BOOST_LOG_TRIVIAL(trace) << "FluidSolver: initFluid";
}


} // Physics

} // d2

} // GooBalls
