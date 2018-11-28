#include "abstract_sph.hpp"
#include "pick_rows.hpp"
#include "generic/is_finite.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

void AbstractSph::prepareFluid(Scene& scene) const {
    assert(scene.fluid.get() != nullptr);
    assert(scene.fluid->sanity_check());
    const auto& pos = scene.fluid->particles_position();
    const auto& vs = scene.fluid->particles_velocity();
    const auto& ms = scene.fluid->particles_mass();
    int PN = pos.rows();
    scene.fluid->fluid_neighborhood->inRange(pos, scene.fluid->h());
    scene.fluid->particles_density().setZero(PN, 1);
    scene.fluid->particles_pressure().setZero(PN, 1);
    scene.fluid->particles_total_force().setZero(PN, 2);
    assert(is_finite(pos));
    assert(is_finite(vs));
    assert(is_finite(ms));
}

void AbstractSph::advance(Scene& scene, TimeStep dt){
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

void AbstractSph::computeGravityForce(const Scene& scene) {
    const auto& pos = scene.fluid->particles_position();
    const auto& rho = scene.fluid->particles_density();
    FGravity.resize(pos.rows(), 2);
    FGravity.col(0) = scene.gravity[0] * rho;
    FGravity.col(1) = scene.gravity[1] * rho;
}

void AbstractSph::computeFluidPressure(Scene& scene) const {
    auto rho0 = scene.fluid->rest_density();
    auto pressure_gamma = scene.fluid->pressure_gamma();
    const auto& rho = scene.fluid->particles_density();
    auto& ps = scene.fluid->particles_pressure();
    auto K = scene.fluid->stiffnessConstant();
    //ps = K * (rho.array() - rho0);
    ps = K * rho0 / pressure_gamma * ((rho.array()/rho0).pow(pressure_gamma) - 1.0);
    ps = ps.array().max(0);
}

void AbstractSph::computeStandardPressureForce(const Scene& scene, const Kernel& pressureKernel){
    const auto& pos = scene.fluid->particles_position();
    const auto& ms = scene.fluid->particles_mass();
    const auto& rho = scene.fluid->particles_density();
    const auto& ps = scene.fluid->particles_pressure();
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    FPressure.resize(PN, 2);
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        Coordinates2d jpos, jGrad;
        pickRows(pos, index, jpos);
        Coordinates2d jPress(index.size(), 2);
        auto xij = -(jpos.rowwise() - pos.row(i));
        pressureKernel.compute(xij, nullptr, &jGrad, nullptr);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            jPress.row(j) = ms[jj] / rho[jj] * (ps[jj] + ps[i]) * jGrad.row(j);
        }
        // f_i^pressure = - sum_j m_j*(p_i + p_j) / (2 rho_j) \nabla W(r_i - r_j, h)
        FPressure.row(i) = (- 1.0/2.0) * jPress.colwise().sum();
    }
}

void AbstractSph::computeMomentumPreservingPressureForce(const Scene& scene, const Kernel& pressureKernel) {
    const auto& pos = scene.fluid->particles_position();
    const auto& ms = scene.fluid->particles_mass();
    const auto& rho = scene.fluid->particles_density();
    const auto& ps = scene.fluid->particles_pressure();
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    FPressure.resize(PN, 2);
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        Coordinates2d jpos, jGrad;
        pickRows(pos, index, jpos);
        Coordinates2d jPress(index.size(), 2);
        auto xij = -(jpos.rowwise() - pos.row(i));
        pressureKernel.compute(xij, nullptr, &jGrad, nullptr);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            jPress.row(j) = ms[jj] * (ps[i]/(rho[i] * rho[i]) + ps[jj]/(rho[jj]*rho[jj])) * jGrad.row(j);
        }
        // f_i^pressure = - m_i * sum_j m_j*(p_i/rho_i^2 + p_j/rho_j^2) \nabla W(r_i - r_j, h)
        FPressure.row(i) = - ms[i] * jPress.colwise().sum();
    }
}


void AbstractSph::computeStandardViscosityForce(const Scene& scene, const Kernel& viscosityKernel) {
    const auto& pos = scene.fluid->particles_position();
    const auto& ms = scene.fluid->particles_mass();
    const auto& vs = scene.fluid->particles_velocity();
    const auto mu = scene.fluid->fluid_viscosity();
    const auto& rho = scene.fluid->particles_density();
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    FViscosity.resize(PN, 2);
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        Coordinates2d jpos;
        pickRows(pos, index, jpos);
        Coordinates1d jLap;
        Coordinates2d jVisc(index.size(), 2);
        auto xij = -(jpos.rowwise() - pos.row(i));
        viscosityKernel.compute(xij, nullptr, nullptr, &jLap);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            jVisc.row(j) = ms[jj] / rho[jj] * (vs.row(jj) - vs.row(i)) * jLap[j];
        }
        // f_i^viscosity = mu * sum_j m_j (v_j - v_i) / rho_j \nabla^2 W(r_i - r_j, h)
        FViscosity.row(i) = mu * jVisc.colwise().sum();
    }
}

void AbstractSph::computeStandardSurfaceTensionForce(const Scene& scene, const Kernel& kernel, FloatPrecision color_relevant_normal_size) {
    const auto& pos = scene.fluid->particles_position();
    const auto& ms = scene.fluid->particles_mass();
    const auto& rho = scene.fluid->particles_density();
    auto color_sigma = scene.fluid->surface_tension();
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    FSurface.setZero(PN, 2);
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        Coordinates2d jpos;
        Coordinates2d jGrad;
        Coordinates1d jCLap;
        Coordinates2d jColGrad(index.size(), 2);
        Coordinates1d jColLap(index.size());
        pickRows(pos, index, jpos);
        // c_s(r_i) = sum_j m_j/rho-j W(r_i - r_j, h)
        auto xij = -(jpos.rowwise() - pos.row(i));
        kernel.compute(xij, nullptr, &jGrad, &jCLap);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            FloatPrecision a = ms[jj] / rho[jj];
            jColGrad.row(j) = a * jGrad.row(j);
            jColLap[j] = a * jCLap[j];
        }
        TranslationVector colorN = jColGrad.colwise().sum();
        FloatPrecision colorLap = jColLap.sum();
        FloatPrecision colorNNorm = colorN.norm();
        if(colorNNorm > color_relevant_normal_size){
            FloatPrecision aa = (-color_sigma * colorLap / colorNNorm);
            FSurface.row(i) = aa * colorN;
        }
    }
}

void AbstractSph::prepareBoundary(Scene& scene) const {
    Coordinates1d& psi = scene.fluid->boundary_psi();
    FloatPrecision rho0 = scene.fluid->rest_density();
    psi = rho0 * scene.fluid->boundary_volume();
    auto h = scene.fluid->h();
    scene.fluid->boundary_neighborhood->inRange(scene.fluid->particles_position(), scene.fluid->boundary_position(), h);
    scene.fluid->boundary_force().setZero(psi.rows(), 2);
}

/// computes rho_i += sum_j m_j W_ij
void AbstractSph::addFluidDensity(Scene& scene, const Kernel& densityKernel) const {
    const auto& pos = scene.fluid->particles_position();
    int PN = pos.rows();
    const auto& ms = scene.fluid->particles_mass();
    auto& rho = scene.fluid->particles_density();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    for(int i = 0; i < PN; ++i){
        Coordinates1d jW;
        Coordinates2d jpos;
        // density from fluid<->fluid
        const auto& index = fluid_index[i];
        // the index should never be empty, as each particles gets at least itself as neighbor
        assert(!index.empty());
        // rho(r_i) = sum_j m_j W(r_i - r_j, h)
        pickRows(pos, index, jpos);
        auto xij = -(jpos.rowwise() - pos.row(i));
        densityKernel.compute(xij, &jW, nullptr, nullptr);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            jW[j] *= ms[jj];
        }
        rho[i] += jW.sum();
    }
}

} // Physics

} // d2

} // GooBalls

