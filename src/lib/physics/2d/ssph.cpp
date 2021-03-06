#include "ssph.hpp"
#include "kernel_debrun_spiky.hpp"
#include "kernel_poly6.hpp"
#include "kernel_viscosity.hpp"
#include "generic/eigen.hpp"
#include "generic/is_finite.hpp"
#include <iostream>
#include <boost/log/trivial.hpp>

namespace GooBalls {

namespace d2 {

namespace Physics {

SSPH::SSPH(){
    // Decent defaults
    m_kernelDensity = std::make_unique<Poly6>();
    m_kernelPressure = std::make_unique<DebrunSpiky>();
    m_kernelViscosity = std::make_unique<Viscosity>();
}

void SSPH::densityKernel(std::unique_ptr<Kernel>&& k) {
    m_kernelDensity = std::move(k);
}
void SSPH::pressureKernel(std::unique_ptr<Kernel>&& k) {
    m_kernelPressure = std::move(k);
}
void SSPH::viscosityKernel(std::unique_ptr<Kernel>&& k) {
    m_kernelViscosity = std::move(k);
}

bool SSPH::considerBoundary(bool consider) {
    m_consider_boundary = consider;
    return true;
}

bool SSPH::considerBoundary() const {
    return m_consider_boundary;
}

void SSPH::computeTotalForce(Scene& scene, TimeStep){
    if(scene.fluid.get() == nullptr || scene.fluid->particles_position().rows() == 0){
        return;
    }
    prepareFluid(scene);
    assert(m_kernelDensity.get() != nullptr);
    assert(m_kernelPressure.get() != nullptr);
    assert(m_kernelViscosity.get() != nullptr);
    const Float h = scene.fluid->h();
    m_kernelDensity->setH(h);
    m_kernelPressure->setH(h);
    m_kernelViscosity->setH(h);
    const auto& pos = scene.fluid->particles_position();
    const auto& vs = scene.fluid->particles_velocity();
    const auto& ms = scene.fluid->particles_mass();
    const auto& ps = scene.fluid->particles_pressure();
    // rho, density: a value measured in kg/m^3, water: 1000, air: 1.3
    // p, pressure: force per unit area
    // nu, kinematic viscosity: high values: fluid doesn't like to deform, low values: fluid likes deformation
    // often: nu = mu / rho
    // mu, dynamic viscosity coefficient:
    Float mu_boundary = scene.fluid->boundary_viscosity(); // viscosity towards wall

    Float visc_epsilon = 0.01;
    const int PN = pos.rows();
    // Müller et al., all equations we need:
    // density: rho(r_i) = sum_j m_j W(r_i - r_j, h)
    // a_i = f_i / rho_i
    // pressure: p = k (rho - rho_0)
    // f_i^pressure = - sum_j m_j (p_i + p_j)/(2 rho_j) \nabla W(r_i - r_j, h)
    // f_i^viscosity = mu * sum_j m_j (v_j - v_i) / rho_j \nabla^2 W(r_i - r_j, h)
    // color field: c_s(r_i) = sum_j m_j/rho_j W(r_i - r_j, h)
    // n = \nabla c_s
    // f^surface = - sigma \nabla^2 c_s n/|n|
    // f_external: given
    //
    // boundary particles:
    // psi(rho_0) = rho_0 * V_bi
    // corrected density of fluid particle:
    // rho_i = m_i * sum_j W_ij + sum_k psi_bk(rho_0) W_ik
    // pressure force from boundary to fluid particle:
    // F^pressure = - m_i sum_k psi_k(rho_0) p_i / (rho_i * rho_k) nabla W_ik
    // F^viscosity = mu * sum_k psi_k(rho_0) (v_k - v_i) / rho_k \nabla^2 W(r_i - r_k, h)
    // surface force should be fine I guess ..
    // implementation:
    // let's get the neighborhood information
    bool consider_boundary = considerBoundary() && scene.fluid->boundary_volume().rows() > 0;

    if(consider_boundary){
        prepareBoundary(scene);
    }
    auto& rho = scene.fluid->particles_density();
    const auto& boundary_index = scene.fluid->boundary_neighborhood->indexes();

    const Coordinates1d& psi = scene.fluid->boundary_psi();

    computeFluidDensity(scene, *m_kernelDensity);
    // density from fluid<->boundary
    if(consider_boundary){
        const auto& bPos = scene.fluid->boundary_position();
        for(int i = 0; i < PN; ++i){
            auto& index = boundary_index[i];
            if(index.empty()){
                continue;
            }
            int N = index.size();
            // adjusted density: m_i * sum_j W_ij + m_i * sum_k W_ik
            // = m_i * sum_j W_ij + sum_k psi_k * W_ik
            // sum_k psi_bk(rho_0) W_ik
            TranslationVector vi = pos.row(i);
            Float jW = 0.0;
            for(int j = 0; j < N; ++j){
                int jj = index[j];
                TranslationVector xik = vi - bPos.row(jj);
                Float w = m_kernelDensity->computeValue(xik);
                jW += psi[jj] * w;
            }
            rho[i] += jW;
        }
    }

    computeFluidPressure(scene);
    computeStandardViscosityForce(scene, *m_kernelViscosity);
    //computeStandardPressureForce(scene, *m_kernelPressure);
    computeMomentumPreservingPressureForce(scene, *m_kernelPressure);
    //computeStandardSurfaceTensionForce(scene, *m_kernelPressure, color_relevant_normal_size);
    if(consider_boundary){
        /*
        if(scene.fluid->boundary_velocity().maxCoeff() > 50){
            BOOST_LOG_TRIVIAL(info) << "largest boundary speed: " << scene.fluid->boundary_velocity().maxCoeff();
        }
        */
        scene.fluid->boundary_force().setZero();
        for(int i = 0; i < PN; ++i){
            // pressure: makes sure stuff doesn't enter the rigid body
            // F^pressure = - m_i sum_k psi_k(rho_0) p_i / (rho_i * rho_k) nabla W_ik
            // F^viscosity = mu * sum_k psi_k(rho_0) (v_k - v_i) / rho_k \nabla^2 W(r_i - r_k, h)
            // paper assumption: rho_k = rho_i
            auto& index = boundary_index[i];
            if(index.empty()){
                continue;
            }
            Coordinates2d jPress(index.size(), 2);
            Coordinates2d jVisc(index.size(), 2);
            Coordinates2d jpos, jGrad;
            Coordinates1d jLap;
            pickRows(scene.fluid->boundary_position(), index, jpos);
            Coordinates2d xik = -(jpos.rowwise() - pos.row(i));
            m_kernelPressure->compute(xik, nullptr, &jGrad, nullptr);
            m_kernelViscosity->compute(xik, nullptr, nullptr, &jLap);
            Float rhoInv = 1.0/rho[i];
            Float mA = -ms[i]*rhoInv*rhoInv*ps[i];
            Float mB = mu_boundary * ms[i]*rhoInv;
            for(size_t j = 0; j < index.size(); ++j){
                int jj = index[j];
                TranslationVector vij = -(scene.fluid->boundary_velocity().row(jj) - vs.row(i));
                jPress.row(j) = mA*psi[jj] * jGrad.row(j);
                Float PI = h * std::max(vij.dot(xik.row(j)), Float(0.0))/(xik.row(j).squaredNorm() + visc_epsilon*h*h);
                jVisc.row(j) = mB * psi[jj] * PI * jGrad.row(j);
                TranslationVector F = scene.fluid->boundary_force().row(jj) - (jPress.row(j) + jVisc.row(j));
                scene.fluid->boundary_force().row(jj) = F;
            }
            FPressure.row(i) += jPress.colwise().sum();
            FViscosity.row(i) += jVisc.colwise().sum();
        }
    }
    /*
    if(scene.fluid->boundary_force().maxCoeff() > 600000){
        BOOST_LOG_TRIVIAL(info) << "maxF: " << scene.fluid->boundary_force().maxCoeff();
    }
    */
    assert(is_finite(FPressure));
    assert(is_finite(FViscosity));
    assert(is_finite(FSurface));
    computeGravityForce(scene);
    scene.fluid->particles_total_force() = FPressure + FViscosity + FSurface + FGravity + scene.fluid->particles_external_force();
}


void SSPH::advance(Scene& scene, TimeStep dt){
    if(scene.fluid.get() == nullptr){
        return;
    }
    computeTotalForce(scene, dt);
    Coordinates2d a;
    // a_i = f_i / rho_i
    const auto& rho = scene.fluid->particles_density();
    const auto& Ftotal = scene.fluid->particles_total_force();
    assert(is_finite(Ftotal));
    assert(is_finite(rho));
    a.resize(rho.rows(), 2);
    a.col(0) = Ftotal.col(0).array() / rho.array();
    a.col(1) = Ftotal.col(1).array() / rho.array();
    assert(is_finite(a));
    auto& pos = scene.fluid->particles_position();
    auto& vs = scene.fluid->particles_velocity();
    vs = vs + dt * a;
    assert(is_finite(vs));
    scene.room.restrictFluid(* scene.fluid);
    pos = pos + dt * vs;
    assert(is_finite(pos));
    limitVelocity(scene);
}


} // Physics

} // d2

} // GooBalls
