#include "ssph.hpp"
#include "spatial/2d/neighborhood_spatial_hashing.hpp"
#include "kernel_debrun_spiky.hpp"
#include "kernel_poly6.hpp"
#include "pick_rows.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

using namespace Spatial;

SSPH::SSPH(){
    m_neighborhood = std::make_unique<NeighborhoodSpatialHashing>();
    m_boundary_neighborhood = std::make_unique<NeighborhoodSpatialHashing>();
    m_kernel = std::make_unique<DebrunSpiky>();
    //m_kernel = std::make_unique<Poly6>();
}

typedef Eigen::Matrix<FloatPrecision, 1, 2> RowVec;

void SSPH::computeTotalForce(Scene& scene, TimeStep dt){
    if(scene.fluid.get() == nullptr){
        return;
    }
    assert(m_kernel.get() != nullptr);
    assert(scene.fluid->sanity_check());
    const CoordinatePrecision h = scene.fluid->h();
    m_kernel->setH(h);
    auto& pos = scene.fluid->particles_position();
    auto& vs = scene.fluid->particles_velocity();
    auto& ms = scene.fluid->particles_mass();
    FloatPrecision K = 100.0; // gas constant, TODO: correct value?
    FloatPrecision rho0 = 200000.0; // rest density, TODO: get correct base value
    FloatPrecision color_relevant_normal_size = 0.01; // TODO: correct value
    FloatPrecision color_sigma = 100.0; // surface tension, TODO: correct value
    FloatPrecision mu = 100.0; // viscosity
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
    m_neighborhood->inRange(scene.fluid->particles_position(), h);
    auto& rho = scene.fluid->particles_density();
    rho.resize(PN);
    Coordinates1d jW(1,1);
    Coordinates2d jpos(1,2);
    Coordinates1d psi;
    bool consider_boundary = m_consider_boundary && scene.fluid->boundary_volume().rows() > 0;
    if(consider_boundary){
        psi = rho0 * scene.fluid->boundary_volume();
        m_boundary_neighborhood->inRange(scene.fluid->particles_position(), scene.fluid->boundary_position(), h);
        assert(m_boundary_neighborhood->indexes().size() == PN);
        m_boundary_force.resize(psi.rows(), Eigen::NoChange);
        m_boundary_force.setZero();
    }
    for(int i = 0; i < PN; ++i){
        // density from fluid<->fluid
        auto& index = m_neighborhood->indexes()[i];
        // rho(r_i) = sum_j m_j W(r_i - r_j, h)
        pickRows(pos, index, jpos);
        m_kernel->compute(-(jpos.rowwise() - pos.row(i)), &jW, nullptr, nullptr);
        for(int j = 0; j < index.size(); ++j){
            int jj = index[j];
            jW[j] *= ms[jj];
        }
        rho[i] = jW.sum();
        // density from fluid<->boundary
        if(consider_boundary){
            auto& index = m_boundary_neighborhood->indexes()[i];
            pickRows(scene.fluid->boundary_position(), index, jpos);
            // adjusted density: m_i * sum_j W_ij + m_i * sum_k W_ik
            // = m_i * sum_j W_ij + sum_k psi_k * W_ik
            // sum_k psi_bk(rho_0) W_ik
            m_kernel->compute(-(jpos.rowwise() - pos.row(i)), &jW, nullptr, nullptr);
            for(int j = 0; j < index.size(); ++j){
                int jj = index[j];
                jW[j] *= psi[jj];
            }
            //rho[i] += jW.sum();
        }
    }

    // pressure: p = k (rho - rho_0)
    Coordinates1d ps = K * (rho.array() - rho0);
    Coordinates2d FPressure (PN, 2);
    Coordinates2d FViscosity (PN, 2);
    Coordinates2d FSurface (PN, 2);
    Coordinates1d colorField(PN, 1);
    Coordinates2d colorFieldN(PN, 2);
    Coordinates2d jGrad;
    Coordinates1d jLap;
    for(int i = 0; i < PN; ++i){
        auto& index = m_neighborhood->indexes()[i];
        pickRows(pos, index, jpos);
        Coordinates2d jPress(index.size(), 2);
        Coordinates2d jVisc(index.size(), 2);
        // TODO: do we need the actual color value at some point? We don't, right?
        //Coordinates1d jColor(index.size());
        Coordinates2d jColGrad(index.size(), 2);
        Coordinates1d jColLap(index.size());
        // f_i^pressure = - sum_j m_j*(p_i + p_j) / (2 rho_j) \nabla W(r_i - r_j, h)
        // f_i^viscosity = mu * sum_j m_j (v_j - v_i) / rho_j \nabla^2 W(r_i - r_j, h)
        // c_s(r_i) = sum_j m_j/rho-j W(r_i - r_j, h)
        m_kernel->compute(-(jpos.rowwise() - pos.row(i)), nullptr /*&jW*/, &jGrad, &jLap);
        for(int j = 0; j < index.size(); ++j){
            int jj = index[j];
            FloatPrecision a = ms[jj] / rho[jj];
            jPress.row(j) = a * (ps[jj] + ps[i]) / 2.0 * jGrad.row(j);
            jVisc.row(j) = a * (vs.row(jj) - vs.row(i)) * jLap[j];
            //jColor[j] = a * jW[j];
            jColGrad.row(j) = a * jGrad.row(j);
            jColLap[j] = a * jLap[j];
        }
        FPressure.row(i) = - jPress.colwise().sum();
        FViscosity.row(i) = mu * jVisc.colwise().sum();
        if(consider_boundary){
            // F^pressure = - m_i sum_k psi_k(rho_0) p_i / (rho_i * rho_k) nabla W_ik
            // F^viscosity = mu * sum_k psi_k(rho_0) (v_k - v_i) / rho_k \nabla^2 W(r_i - r_k, h)
            // paper assumption: rho_k = rho_i
            auto& index = m_boundary_neighborhood->indexes()[i];
            Coordinates2d jPress(index.size(), 2);
            Coordinates2d jVisc(index.size(), 2);
            pickRows(scene.fluid->boundary_position(), index, jpos);
            m_kernel->compute(jpos.rowwise() - pos.row(i), nullptr, &jGrad, &jLap);
            for(int j = 0; j < index.size(); ++j){
                int jj = index[j];
                jPress.row(j) = psi[jj] * ps[i]/(rho[i] * rho[i]) * jGrad.row(j);
                jVisc.row(j) = psi[jj] * (scene.fluid->boundary_velocity().row(jj) - vs.row(i))/rho[i] * jLap[j];
                m_boundary_force.row(jj) = m_boundary_force.row(jj) - (jPress.row(j) + jVisc.row(j));
            }
            //FPressure.row(i) += - ms[i] * jPress.colwise().sum();
            FViscosity.row(i) += mu * jVisc.colwise().sum();
        }
        //FloatPrecision color = jColor.sum();
        RowVec colorN = jColGrad.colwise().sum();
        FloatPrecision colorLap = jColLap.sum();
        FloatPrecision colorNNorm = colorN.norm();
        if(colorNNorm > color_relevant_normal_size){
            FloatPrecision aa = (-color_sigma * colorLap / colorNNorm);
            FSurface.row(i) = aa * colorN;
        }
    }
    //scene.fluid->particles_total_force() = (FPressure + FViscosity + FSurface).rowwise() + scene.gravity;
    scene.fluid->particles_total_force() = FViscosity + FPressure + FSurface;
}


} // Physics

} // d2

} // GooBalls
