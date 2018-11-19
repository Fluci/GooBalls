#include "visco_elastic.hpp"
#include "ssph.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

ViscoElastic::ViscoElastic(){
    m_solver = std::make_unique<SSPH>();
}

void ViscoElastic::advance(Scene& scene, TimeStep dt){
    if(scene.fluid.get() == nullptr){
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

    // v^* = v^t + dt * a;
    // Delta v =
    vs = vs + dt * a;
    pos = pos + dt * vs;
}

void ViscoElastic::computeTotalForce(Scene& scene, TimeStep dt){
    m_solver->computeTotalForce(scene, dt);
}

} // Physics

} // d2

} // GooBalls
