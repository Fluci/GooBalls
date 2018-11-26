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
    assert(is_finite(pos));
    assert(is_finite(vs));
    assert(is_finite(ms));
    FloatPrecision K = scene.fluid->K(); // gas constant dependent on temperature, TODO: correct value?

    // p_i = k rho0 / gamma ((rho_i/rho0)^gamma - 1)
    // F^p_i = m_i sum_j m_j ( p_i / rho_i^2 + p_j / rho_j^2) \nabbla W_ij
    // v^adv_i = v_i + dt * F^adv_i / m_i
    // rho^adv_i = rho_i + dt * sum_j m_j v^adv_ij \nabbla W_ij
    // dt^2 sum_j (m_j/m_i F^p_i - F^p_j) * \nabbla W_ij = rho0 - rho^adv_i
    // sum_j a_ij p_j = b_i = rho_0 - rho^adv_i

    scene.fluid->fluid_neighborhood->inRange(pos, h);
    predictAdvection(scene, dt, *m_kernelDensity);
    pressureSolve(scene, dt, *m_kernelDensity);
    computeMomentumPreservingPressureForce(scene, *m_kernelDensity);
    //computeStandardPressureForce(scene, *m_kernelDensity);

    FPressure = FPressure.array().min(100*K).max(-100*K);
    FViscosity = FViscosity.array().min(100*K).max(-100*K);
    assert(is_finite(FPressure));
    assert(is_finite(FViscosity));
    assert(is_finite(FSurface));
    scene.fluid->particles_total_force() = FPressure + FGravity + FViscosity;
}


void IISPH::predictAdvection(Scene& scene, TimeStep dt, const Kernel& kernel) {
    const auto& pos = scene.fluid->particles_position();
    const auto& vs = scene.fluid->particles_velocity();
    const auto& ps = scene.fluid->particles_density();
    const auto& ms = scene.fluid->particles_mass();
    const auto& rho = scene.fluid->particles_density();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    int PN = vs.rows();
    // for all particle i do
    //     compute rho_i(t) = sum_j m_j W_ij
    addFluidDensity(scene, *m_kernelDensity);
    //     compute F^adv_i = F^SurfaceTension_i + F^Gravity_i + F^Visco_i + ...
    computeGravityForce(scene);
    computeStandardViscosityForce(scene, *m_kernelViscosity);
    // TODO: computeSurfaceTensionForce();
    const auto Fadv = FGravity + FViscosity;
    //     predict v^adv_i = v_i + dt F^adv_i / m_i
    Coordinates2d da = Fadv.array().colwise() / ms.array();
    Coordinates2d Vadv = vs + dt*da;
    //     d_ii = - dt^2 sum_j m_j/rho_i^2 \nabbla W_ij
    dii.resize(PN, 2);
    auto dt2 = dt*dt;
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        Coordinates2d diis(index.size(), 2);
        Coordinates2d jpos;
        Coordinates2d wGrad;
        pickRows(pos, index, jpos);
        auto xij = -(jpos.rowwise() - pos.row(i));
        kernel.compute(xij, nullptr, &wGrad, nullptr);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            diis.row(j) = ms[jj] * wGrad.row(j);
        }
        dii.row(i) = - dt2/(rho[i]*rho[i]) * diis.colwise().sum();
    }

    // for all particle i do
    // rho^adv_i = rho_i + dt sum_j m_j * v^adv_ij * \nabbla W_ij
    rhoAdv.resize(PN, 1);
    // initializing pi is a bit of an art
    // these are possibilities, last chosen by paper:
    // p^0_i = 0
    // p^0_i = p_i(t-dt).
    // p^0_i = 0.5 p_i(t - dt)
    p0 = 0.5* ps;
    aii.resize(PN, 1);
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        Coordinates1d rhos(index.size(), 1);
        Coordinates2d jpos;
        Coordinates2d wGrad;
        pickRows(pos, index, jpos);
        auto xij = -(jpos.rowwise() - pos.row(i));
        kernel.compute(xij, nullptr, &wGrad, nullptr);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            rhos[j] = ms[jj] * (Vadv.row(i) - Vadv.row(jj)).dot(wGrad.row(j));
        }
        rhoAdv[i] = rho[i] + dt*rhos.sum();
        // compute a_ii = sum_j m_j (d_ii - d_ji) nabbla W_ij
        Coordinates1d aiis (index.size(), 1);
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            // d_ij = - dt^2 * m_j / rho_j^2 \nabbla W_ij
            TranslationVector dji = -dt2 * ms[jj] / (rho[jj]*rho[jj]) * wGrad.row(j);
            aiis[j] = ms[jj] * (dii.row(i) - dji).dot(wGrad.row(j));
        }
        aii[i] = aiis.sum();
    }
}

void IISPH::pressureSolve(Scene& scene, TimeStep dt, const Kernel& kernel) {
    // rho^l_avg = 1/n * sum_i rho^l_i
    // l = 0
    int l = 0;
    // while rho^l_avg - rho0 > eta OR l < 2 do
    // for all particle i do:
    //    sum_j d_ij p^l_j = dt^2 sum_j - m_j / rho_j^2 p^l_j \nabbla _Wij
    // for all particle i do:
    //    comptue p^(l+1)_i = (1 - omega) p^l_i + omega/a_ii ( rho0 - rho^adv_i - sum_j m_j (sum_j d_ij p^l_j - d_jj p^l_j - sum_{k!= i} d_ji p^l_k ) \nabbla W_ij)
    //    p_i(t) = p_i^l
    // l = l + 1
    const auto& ms = scene.fluid->particles_mass();
    const auto& rho = scene.fluid->particles_density();
    const auto& pos = scene.fluid->particles_position();
    auto& ps = scene.fluid->particles_pressure();
    auto rho0 = scene.fluid->rest_density();
    auto PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    FloatPrecision eta = 0.01*rho0;
    Coordinates1d p1;
    Coordinates1d rhol;
    Coordinates2d dp; // sum_j dij p^l_j, for given i
    p1.resize(PN, 1);
    rhol.resize(PN, 1);
    dp.resize(PN, 2);
    FloatPrecision omega = 0.99; // relaxation factor
    do {
        //    sum_j d_ij p^l_j = - dt^2 sum_j m_j / rho_j^2 p^l_j \nabbla _Wij
        for(int i = 0; i < PN; ++i){
            const auto& index = fluid_index[i];
            Coordinates2d jpos;
            pickRows(pos, index, jpos);
            Coordinates2d wGrad;
            Coordinates2d xij = -(jpos.rowwise() - pos.row(i));
            kernel.compute(xij, nullptr, &wGrad, nullptr);
            Coordinates2d dps(index.size(), 2);
            for(size_t j = 0; j < index.size(); ++j){
                int jj = index[j];
                dps.row(j) = ms[jj] / (rho[jj] * rho[jj]) * p0[jj] * wGrad.row(j);
            }
            dp.row(i) = -dt*dt * dps.colwise().sum();
        }
        //    comptue p^(l+1)_i = (1 - omega) p^l_i + omega/a_ii (rho0 - rho^adv_i - A_i)
        //         A_i = sum_j m_j (dp_i - d_jj p^l_j - dpp_j ) \nabbla W_ij
        //             dp_i = sum_j d_ij p^l_j
        //             dpp_j = sum_{k!= i} d_jk p^l_k = sum_k d_jk p_k^l - d_ji p_i^l
        //             sum_k d_jk p_k^l = dp.row(j)
        //             d_ij = -dt^2 m_j / rho_j^2 \nabbla W_ij
        // compute A_j and store in p1[i]
        for(int i = 0; i < PN; ++i){
            const auto& indexI = fluid_index[i];
            Coordinates2d jpos;
            pickRows(pos, indexI, jpos);
            Coordinates2d wGradI;
            Coordinates2d xij = -(jpos.rowwise() - pos.row(i));
            kernel.compute(xij, nullptr, &wGradI, nullptr);
            Coordinates1d Ais(indexI.size());
            for(size_t j = 0; j < indexI.size(); ++j){
                int jj = indexI[j];
                TranslationVector dppj = dp.row(jj) - (-dt*dt*ms[jj]/(rho[jj]*rho[jj]) * wGradI.row(j) * p0[i]);
                Ais[j] = ms[jj]*(dp.row(i) - dii.row(jj)*p0[jj] - dppj).dot(wGradI.row(j));
            }
            p1[i] = Ais.sum();
        }
        p1 = (1.0 - omega) * p0.array() + omega/aii.array() * (rho0 - rhoAdv.array() - p1.array());
        std::swap(p0, p1);
        l++;
    } while (rhol.mean() - rho0 > eta || l < 2);
    //    p_i(t) = p_i^(l+1)
    ps = p0;
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
