#include "iisph.hpp"
#include "spatial/2d/neighborhood_spatial_hashing.hpp"
#include "kernel_debrun_spiky.hpp"
#include "kernel_poly6.hpp"
#include "kernel_viscosity.hpp"
#include "pick_rows.hpp"
#include <iostream>
#include "generic/is_finite.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

using namespace Spatial;

IISPH::IISPH(){
    // Decent defaults
    m_kernelDensity = std::make_unique<Poly6>();
    m_kernelPressure = std::make_unique<DebrunSpiky>();
    m_kernelViscosity = std::make_unique<Viscosity>();
}

void IISPH::densityKernel(std::unique_ptr<Kernel>&& k) {
    m_kernelDensity = std::move(k);
}
void IISPH::pressureKernel(std::unique_ptr<Kernel>&& k) {
    m_kernelPressure = std::move(k);
}
void IISPH::viscosityKernel(std::unique_ptr<Kernel>&& k) {
    m_kernelViscosity = std::move(k);
}

void IISPH::computeTotalForce(Scene& scene, TimeStep dt){
    if(scene.fluid.get() == nullptr){
        return;
    }
    prepareFluid(scene);
    assert(m_kernelDensity.get() != nullptr);
    assert(m_kernelPressure.get() != nullptr);
    assert(m_kernelViscosity.get() != nullptr);
    const CoordinatePrecision h = scene.fluid->h();
    m_kernelDensity->setH(h);
    m_kernelPressure->setH(h);
    m_kernelViscosity->setH(h);
    auto& pos = scene.fluid->particles_position();
    auto& vs = scene.fluid->particles_velocity();
    auto& ms = scene.fluid->particles_mass();
    auto& ps = scene.fluid->particles_pressure();
    assert(is_finite(pos));
    assert(is_finite(vs));
    assert(is_finite(ms));
    FloatPrecision K = scene.fluid->K(); // gas constant dependent on temperature, TODO: correct value?
    // rho, density: a value measured in kg/m^3, water: 1000, air: 1.3
    // p, pressure: force per unit area
    // nu, kinematic viscosity: high values: fluid doesn't like to deform, low values: fluid likes deformation
    // often: nu = mu / rho
    // mu, dynamic viscosity coefficient:
    FloatPrecision rho0 = scene.fluid->rest_density(); // rest density? according to Bridson: environmental pressure?, TODO: get correct base value
    FloatPrecision color_sigma = scene.fluid->surface_tension(); // surface tension, TODO: correct value
    FloatPrecision mu = scene.fluid->fluid_viscosity(); // viscosity
    FloatPrecision mu_boundary = scene.fluid->boundary_viscosity(); // viscosity towards wall
    FloatPrecision pressure_gamma = scene.fluid->pressure_gamma(); // 1..7


    // p_i = k rho0 / gamma ((rho_i/rho0)^gamma - 1)
    // F^p_i = m_i sum_j m_j ( p_i / rho_i^2 + p_j / rho_j^2) \nabbla W_ij
    // v^adv_i = v_i + dt * F^adv_i / m_i
    // rho^adv_i = rho_i + dt * sum_j m_j v^adv_ij \nabbla W_ij
    // dt^2 sum_j (m_j/m_i F^p_i - F^p_j) * \nabbla W_ij = rho0 - rho^adv_i
    // sum_j a_ij p_j = b_i = rho_0 - rho^adv_i

    scene.fluid->fluid_neighborhood->inRange(pos, h);
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    predictAdvection(scene, dt);
    pressureSolve(scene, dt);


    FPressure = FPressure.array().min(100*K).max(-100*K);
    FViscosity = FViscosity.array().min(100*K).max(-100*K);
    assert(is_finite(FPressure));
    assert(is_finite(FViscosity));
    assert(is_finite(FSurface));
    scene.fluid->particles_total_force();
}


void IISPH::predictAdvection(Scene& scene, TimeStep dt) {
    const auto& vs = scene.fluid->particles_velocity();
    const auto& ms = scene.fluid->particles_mass();
    // for all particle i do
    //     compute rho_i(t) = sum_j m_j W_ij
    addFluidDensity(scene, *m_kernelDensity);
    //     compute F^adv_i = F^SurfaceTension_i + F^Gravity_i + F^Visco_i + ...
    computeGravityForce(scene);
    computeStandardViscosityForce(scene, *m_kernelViscosity);
    // TODO: computeSurfaceTensionForce();
    auto Fadv = FGravity + FViscosity;
    //     predict v^adv_i = v_i + dt F^adv_i / m_i
    //auto Vadv = vs + dt * Fadv.array() / ms.array();
    //     d_ii = - dt^2 sum_j m_j/rho_i^2 \nabbla W_ij

    // for all particle i do
    // rho^adv_i = rho_i + dt sum_j m_j * v^adv_ij * \nabbla W_ij
    // p^0_i = 0.5 p_i(t - dt)
    // compute a_ij = sum_j m_j (d_ii - d_ji) nabbla W_ij
}

void IISPH::pressureSolve(Scene& scene, TimeStep dt) {
    // l = 0
    // while rho^l_avg - rho0 > eta AND l < s do
    // for all particle i do: sum_j d_ij p^l_j = dt^2 sum_j - m_j / rho_j^2 p^l_j \nabbla _Wij
    // for all particle i do:
    //    comptue p^(l+1)_i = (1 - omega) p^l_i + omega/a_ii ( rho0 - rho^adv_i - sum_j m_j (sum_j d_ij p^l_j - d_jj p^l_j - sum_{k!= i} d_ji p^l_k ) \nabbla W_ij)
    //    p_i(t) = p_i^l
    // l = l + 1
}

void IISPH::advance(Scene& scene, TimeStep dt){
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
    const auto& ms = scene.fluid->particles_mass();
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


} // Physics

} // d2

} // GooBalls
