#include "visco_elastic.hpp"
#include "ssph.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

ViscoElastic::ViscoElastic(){
    m_solver = std::make_unique<SSPH>();
}

void ViscoElastic::base(std::unique_ptr<FluidSolver>&& b) {
    m_solver = std::move(b);
}

bool ViscoElastic::considerBoundary(bool consider) {
    return m_solver->considerBoundary(consider);
}

bool ViscoElastic::considerBoundary() const {
    return m_solver->considerBoundary();
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
    // D_ij = max(||x_ij|| - r_ij, 0)
    // Delta v_i = - 1/dt * sum_j (c_i + c_j)/2 * m_j/(m_i+m_j) D_ij x_ij / ||x_ij||
    const auto& cs = scene.fluid->particles_velocity_correction();
    const auto& ms = scene.fluid->particles_mass();
    Coordinates2d dx;
    dx.resize(pos.rows(), Eigen::NoChange);
    dx.setZero();
    for(int i = 0; i < pos.rows(); ++i){
        const auto& conn = scene.fluid->particles_connectivity()[i];
        TranslationVector sum;
        sum.setZero();
        for(size_t j = 0; j < conn.size(); ++j){
            auto xij = pos.row(i) - pos.row(conn[j].partner);
            auto xijN = xij.norm();
            auto Dij = std::max(xijN - conn[j].rij, 0.0);
            sum = sum + (cs[i] + cs[j])/2.0 * ms[j] / (ms[i] + ms[j]) * Dij * xij.normalized();
        }
        dx.row(i) = sum;
    }
    Coordinates2d Dv = -dx/dt;
    vs = vs + dt * a + Dv;
    scene.room.restrictFluid(* scene.fluid);
    pos = pos + dt * vs;
}

void ViscoElastic::computeTotalForce(Scene& scene, TimeStep dt){
    m_solver->computeTotalForce(scene, dt);
}

} // Physics

} // d2

} // GooBalls
