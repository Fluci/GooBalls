#include "abstract_sph.hpp"
#include "generic/is_finite.hpp"
#include "generic/eigen.hpp"

#include <boost/log/trivial.hpp>

namespace GooBalls {

namespace d2 {

namespace Physics {

#define INLINE_KERNELS 1

void AbstractSph::initFluid(Scene& scene) {
    BOOST_LOG_TRIVIAL(trace) << "AbstractSph: initializing";
    const auto& pos = scene.fluid->particles_position();
    int PN = pos.rows();
    BOOST_LOG_TRIVIAL(trace) << "Found " << PN << " fluid particles.";
    if(PN != scene.fluid->particles_velocity().rows()){
        // by default, give the particles no speed
        BOOST_LOG_TRIVIAL(warning) << "No proper fluid particle velocity set, setting to zero.";
        scene.fluid->particles_velocity().setOnes(PN, Eigen::NoChange);
    }
    if(PN != scene.fluid->particles_mass().rows()){
        BOOST_LOG_TRIVIAL(warning) << "No proper fluid particle mass set, setting to ones.";
        scene.fluid->particles_mass().setOnes(PN);
    }
    if(PN != scene.fluid->particles_velocity_correction().rows()){
        BOOST_LOG_TRIVIAL(warning) << "No proper fluid particle velocity correction coefficients set, setting to 1.";
        scene.fluid->particles_velocity_correction().resize(PN, Eigen::NoChange);
        scene.fluid->particles_velocity_correction().array() = 0.001;
    }
    if(PN != scene.fluid->particles_external_force().rows()){
        BOOST_LOG_TRIVIAL(warning) << "No proper external particle force set, setting zero.";
        scene.fluid->particles_external_force().setZero(PN, 2);
    }


    scene.fluid->particles_density().setZero(PN, 1);
    scene.fluid->particles_total_force().setZero(PN, 2);
    scene.fluid->particles_pressure().setZero(PN, 1);

    scene.fluid->boundary_force().setZero(scene.fluid->boundary_position().rows(), 2);

    FGravity.setZero(PN, 2);
    FPressure.setZero(PN, 2);
    FSurface.setZero(PN, 2);
    FViscosity.setZero(PN, 2);


    // initialize with some rough values such that the first frame looks ok
    assert(scene.fluid->rest_density() > 0);
    scene.fluid->particles_density().array() = scene.fluid->rest_density();
    computeFluidPressure(scene);
    BOOST_LOG_TRIVIAL(trace) << "Abstract fluid initialized";
}

void AbstractSph::prepareFluid(Scene& scene) const {
    assert(scene.fluid.get() != nullptr);
    assert(scene.fluid->sanity_check());
    const auto& pos = scene.fluid->particles_position();
    const auto& vs = scene.fluid->particles_velocity();
    const auto& ms = scene.fluid->particles_mass();
    assert(is_finite(pos));
    assert(is_finite(vs));
    assert(is_finite(ms));
    scene.fluid->fluid_neighborhood->inRange(pos, scene.fluid->h());
    scene.fluid->particles_density().setZero();
    scene.fluid->particles_pressure().setZero();
    scene.fluid->particles_total_force().setZero();
}

void AbstractSph::prepareBoundary(Scene& scene) const {
    Coordinates1d& psi = scene.fluid->boundary_psi();
    Float rho0 = scene.fluid->rest_density();
    psi = rho0 * scene.fluid->boundary_volume();
    auto h = scene.fluid->h();
    scene.fluid->boundary_neighborhood->inRange(scene.fluid->particles_position(), scene.fluid->boundary_position(), h);
    scene.fluid->boundary_force().setZero();
}

void AbstractSph::advance(Scene& scene, TimeStep dt){
    if(scene.fluid.get() == nullptr){
        return;
    }
    computeTotalForce(scene, dt);
    // a_i = f_i / rho_i
    const auto& rho = scene.fluid->particles_density();
    const auto& Ftotal = scene.fluid->particles_total_force();
    Coordinates2d a(rho.rows(), 2);
    a.col(0) = Ftotal.col(0).array() / rho.array();
    a.col(1) = Ftotal.col(1).array() / rho.array();
    auto& pos = scene.fluid->particles_position();
    auto& vs = scene.fluid->particles_velocity();
    vs = vs + dt * a;
    scene.room.restrictFluid(* scene.fluid);
    pos = pos + dt * vs;
    limitVelocity(scene);
    assert(is_finite(pos));
    assert(is_finite(vs));
}

void AbstractSph::computeGravityForce(const Scene& scene) {
    const auto& rho = scene.fluid->particles_density();
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
    assert(is_finite(ps));
}

void AbstractSph::computeStandardPressureForce(const Scene& scene, const Kernel& pressureKernel){
    const auto& pos = scene.fluid->particles_position();
    const auto& ms = scene.fluid->particles_mass();
    const auto& rho = scene.fluid->particles_density();
    const auto& ps = scene.fluid->particles_pressure();
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
    Coordinates2d jpos, jGrad, jPress;
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        minSize(jPress, index.size());
        pickRows(pos, index, jpos);
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
    assert(is_finite(pos));
    assert(is_finite(ms));
    assert(is_finite(rho));
    assert(is_finite(ps));
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
#if INLINE_KERNELS
    Float h = scene.fluid->h();
    Float kernelScale = pressureKernel.scale2d();
    Float epsilon = 0.000001;
#endif
    Coordinates2d jPress;
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        int N = index.size();
        minSize(jPress, N);
        Float A = ps[i]/(rho[i] * rho[i]);
        for(size_t j = 0; j < N; ++j){
            int jj = index[j];
            TranslationVector xij = -(pos.row(jj) - pos.row(i));
#if INLINE_KERNELS
            auto r1 = xij.rowwise().norm().array();
            TranslationVector jGrad = (-3*kernelScale) * (xij.array().colwise() *( (h - r1).pow(2.0) / (r1.array() + h*epsilon)));
#else
            TranslationVector jGrad = pressureKernel.computeGradient(xij);
#endif
            jPress.row(j) = ms[jj] * (A + ps[jj]/(rho[jj]*rho[jj])) * jGrad;
        }
        // f_i^pressure = - m_i * sum_j m_j*(p_i/rho_i^2 + p_j/rho_j^2) \nabla W(r_i - r_j, h)
        FPressure.row(i) = - ms[i] * jPress.block(0,0,N,2).colwise().sum();
    }
    assert(is_finite(FPressure));
}


void AbstractSph::computeStandardViscosityForce(const Scene& scene, const Kernel& viscosityKernel) {
    const auto& pos = scene.fluid->particles_position();
    const auto& ms = scene.fluid->particles_mass();
    const auto& vs = scene.fluid->particles_velocity();
    const auto mu = scene.fluid->fluid_viscosity();
    const auto& rho = scene.fluid->particles_density();
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
#if INLINE_KERNELS
    Float h = scene.fluid->h();
    Float kernelScale = viscosityKernel.scale2d();
#endif
    Coordinates2d jVisc;
    for(int i = 0; i < PN; ++i){
        const auto& index = fluid_index[i];
        int N = index.size();
        minSize(jVisc, N);
        TranslationVector vi = pos.row(i);

        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            auto xij = -(pos.row(jj) - vi);
#if INLINE_KERNELS
            Float jLap = kernelScale * (h - xij.norm());
#else
            Float jLap = viscosityKernel.computeLaplacian(xij);
#endif
            jVisc.row(j) = ms[jj] / rho[jj] * (vs.row(jj) - vs.row(i)) * jLap;
        }
        // f_i^viscosity = mu * sum_j m_j (v_j - v_i) / rho_j \nabla^2 W(r_i - r_j, h)
        FViscosity.row(i) = mu * jVisc.block(0,0,N,2).colwise().sum();
    }
}

void AbstractSph::computeStandardSurfaceTensionForce(const Scene& scene, const Kernel& kernel, Float color_relevant_normal_size) {
    const auto& pos = scene.fluid->particles_position();
    const auto& ms = scene.fluid->particles_mass();
    const auto& rho = scene.fluid->particles_density();
    auto color_sigma = scene.fluid->surface_tension();
    int PN = pos.rows();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
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
            Float a = ms[jj] / rho[jj];
            jColGrad.row(j) = a * jGrad.row(j);
            jColLap[j] = a * jCLap[j];
        }
        TranslationVector colorN = jColGrad.colwise().sum();
        Float colorLap = jColLap.sum();
        Float colorNNorm = colorN.norm();
        if(colorNNorm > color_relevant_normal_size){
            Float aa = (-color_sigma * colorLap / colorNNorm);
            FSurface.row(i) = aa * colorN;
        }
    }
}

/// computes rho_i += sum_j m_j W_ij
void AbstractSph::addFluidDensity(Scene& scene, const Kernel& densityKernel) const {
    const auto& pos = scene.fluid->particles_position();
    int PN = pos.rows();
    const auto& ms = scene.fluid->particles_mass();
    assert(ms.minCoeff() > 0.0);
    auto& rho = scene.fluid->particles_density();
    const auto& fluid_index = scene.fluid->fluid_neighborhood->indexes();
#if INLINE_KERNELS
    Float h = scene.fluid->h();
    Float kernelScale = densityKernel.scale2d();
    Float h2 = h * h;
#endif
    Coordinates1d jW;
    for(int i = 0; i < PN; ++i){
        // density from fluid<->fluid
        const auto& index = fluid_index[i];
        int N = index.size();
        minSize(jW, N);
        TranslationVector vi = pos.row(i);
        // the index should never be empty, as each particles gets at least itself as neighbor
        //assert(!index.empty());
        // rho(r_i) = sum_j m_j W(r_i - r_j, h)
        /*pickRows(pos, index, jpos);
        auto xij = -(jpos.rowwise() - pos.row(i));
        densityKernel.compute(xij, &jW, nullptr, nullptr);
        */
        for(size_t j = 0; j < N; ++j){
            int jj = index[j];
            TranslationVector xij = -(pos.row(jj) - vi);
#if INLINE_KERNELS
            Float d = h2 - xij.squaredNorm();
            Float v = kernelScale * d*d*d;
#else
            Float v = densityKernel.computeValue(xij);
#endif
            jW[j] = ms[jj] * v;
        }
        rho[i] += jW.block(0,0,N,1).sum();
    }
    assert(rho.minCoeff() > 0.0);
    assert(is_finite(rho));
}

/// based on CFL condition: dt = lambda * h /max(sqrt(K), v_max)
void AbstractSph::limitVelocity(const Scene& scene) const {
    const Float vMax = std::sqrt(scene.fluid->stiffnessConstant());
    auto& vs = scene.fluid->particles_velocity();

    Coordinates1d vsMag2 = vs.rowwise().squaredNorm();
    Float maxV2 = vsMag2.maxCoeff();
    if(maxV2 > vMax*vMax){
        BOOST_LOG_TRIVIAL(info) << "Limiting particle speed from " << std::sqrt(maxV2) << " to " << vMax;
        for(int i = 0; i < vs.rows(); ++i){
            if(vsMag2[i] > vMax*vMax){
                vs.row(i) = vs.row(i) / std::sqrt(vsMag2[i]) * vMax;
            }
        }
        vs.array().min(vMax).max(-vMax);
    }
}


} // Physics

} // d2

} // GooBalls

