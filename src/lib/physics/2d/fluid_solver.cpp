#include "fluid_solver.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

void FluidSolver::advance(Scene& scene, TimeStep dt){
    if(scene.fluid.get() == nullptr){
        return;
    }
    if(scene.fluid->particles_position().rows() == 0){
        return;
    }
    computeTotalForce(scene, dt);
    Coordinates2d a;
    // a_i = f_i / rho_i
    const auto& rho = scene.fluid->particles_density();
    const auto& Ftotal = scene.fluid->particles_total_force();
    a.resize(rho.rows(), 2);
    a.col(0) = Ftotal.col(0).array() / rho.array();
    a.col(1) = Ftotal.col(1).array() / rho.array();
    auto& pos = scene.fluid->particles_position();
    auto& vs = scene.fluid->particles_velocity();
    vs = vs + dt * a;
    scene.room.restrictFluid(* scene.fluid);
    pos = pos + dt * vs;
}

Coordinates2d FluidSolver::gravityForce(const Scene& scene) const {
    const auto& pos = scene.fluid->particles_position();
    const auto& rho = scene.fluid->particles_density();
    Coordinates2d FGravity(pos.rows(), 2);
    FGravity.col(0) = scene.gravity[0] * rho;
    FGravity.col(1) = scene.gravity[1] * rho;
    return FGravity;
}

} // Physics

} // d2

} // GooBalls
