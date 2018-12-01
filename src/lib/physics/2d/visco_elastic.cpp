#include "visco_elastic.hpp"
#include "ssph.hpp"

#include <boost/log/trivial.hpp>
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
    controlConnections(scene);
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
            int jj = conn[j].partner;
            auto xij = pos.row(i) - pos.row(jj);
            auto xijN = xij.norm();
            auto Dij = std::max(xijN - conn[j].rij, 0.0);
            sum = sum + (cs[i] + cs[jj])/2.0 * ms[jj] / (ms[i] + ms[jj]) * Dij * xij.normalized();
        }
        dx.row(i) = sum;
    }

    const auto& bPos = scene.fluid->boundary_position();
    bool consider_boundary = considerBoundary() && bPos.rows() > 0;
    if(consider_boundary){
        Coordinates2d dy;
        dy.resize(pos.rows(), Eigen::NoChange);
        dy.setZero();
        auto h = scene.fluid->h();
        auto alpha = scene.fluid->boundary_merge_threshold();
        scene.fluid->boundary_neighborhood->inRange(pos, bPos, alpha*h);
        const auto& bCs = scene.fluid->boundary_velocity_correction_coefficient();
        const auto& bMs = scene.fluid->boundary_mass();
        for(int i = 0; i < pos.rows(); ++i){
            const auto& index = scene.fluid->boundary_neighborhood->indexes()[i];
            TranslationVector sum;
            sum.setZero();
            for(size_t j = 0; j < index.size(); ++j){
                int jj = index[j];
                auto xij = pos.row(i) - bPos.row(jj);
                auto xijN = xij.norm();
                Float Dij = std::max(xijN - 0.5*alpha*h, 0.0);
                Float M;
                if(bMs[jj] > 0){
                    M =  bMs[jj] / (ms[i] + bMs[jj]);
                } else {
                    // mass equal to zero means it is a static object, hence m_i = infinity
                    M = 1.0/ms[i];
                }
                TranslationVector DX = (cs[i] + bCs[jj])/2.0 * M * Dij * xij.normalized();
                sum = sum + DX;
            }
            dy.row(i) = sum;
        }
        dx = dx + dy;
    }
    Coordinates2d Dv = -dx/dt;
    vs = vs + dt * a + Dv;
    pos = pos + dt * vs;
    scene.room.restrictFluid(* scene.fluid);
    //updateVelocityCorrectionCoefficients(scene, dt);
}

void ViscoElastic::updateVelocityCorrectionCoefficients(Scene& scene, TimeStep dt){
    // D_ij = max(||x_ij|| - r_ij, 0)
    // if (gamma_i + gamma_j)/2 < D_ij/h:
    // c^t+1_i = max(c^t_i - dt * sum_j (w_i + w_j)/2, l_i)
    auto& cs = scene.fluid->particles_velocity_correction();
    const auto l = scene.fluid->particles_lower_velocity_correction_limit();
    const auto w = scene.fluid->particles_weakening_speed();
    const auto gamma = scene.fluid->particles_yield_criterion();
    const auto& pos = scene.fluid->particles_position();
    const auto h = scene.fluid->h();
    Float lowerDij = h*(gamma + gamma)/2; // in the paper, this has to be evaluated for each point pair, we assume gamma_i = gamma_j

    if(w <= 0){
        return;
    }
    for(int i = 0; i < pos.rows(); ++i){
        const auto& conn = scene.fluid->particles_connectivity()[i];
        Float sum = 0.0;
        for(size_t j = 0; j < conn.size(); ++j){
            auto xij = pos.row(i) - pos.row(conn[j].partner);
            auto xijN = xij.norm();
            auto Dij = std::max(xijN - conn[j].rij, 0.0);
            if(lowerDij >= Dij){
                continue;
            }
            // consder to update c_i
            sum += w; // (w_i + w_j)/2
        }
        cs[i] = std::max(cs[i] - dt * sum, l);
        if(dt*sum > 0){
            BOOST_LOG_TRIVIAL(info) << cs[i];
        }
    }
}

void ViscoElastic::controlConnections(Scene& scene) {
    // if ||x_ij|| < h * alpha -> new connection
    // if ||x_ij|| > h * beta -> split
    const auto alpha = scene.fluid->merge_threshold();
    const auto beta = scene.fluid->split_threshold();
    assert(alpha < beta);
    const auto& pos = scene.fluid->particles_position();
    const auto h = scene.fluid->h();
    auto& cs = scene.fluid->particles_connectivity();
    cs.resize(pos.rows());
    // remove connections that are too large
    Float hb2 = h*h*beta*beta;
    int removed = 0;
    for(int i = 0; i < pos.rows(); ++i){
        int next = 0;
        for(size_t j = 0; j < cs[i].size(); ++j){
            TranslationVector xij = pos.row(i) - pos.row(cs[i][j].partner);
            if(xij.squaredNorm() > hb2){
                // remove
                removed++;
            } else {
                // move to next
                cs[i][next] = cs[i][j];
                next++;
            }
        }
        cs[i].resize(next);
        std::sort(cs[i].begin(), cs[i].end(), [](auto a, auto b){return a.partner < b.partner;});
    }
    // add new connections
    scene.fluid->fluid_neighborhood->inRange(pos, h*alpha);
    const auto& indexes = scene.fluid->fluid_neighborhood->indexes();
    int added = 0;
    for(int i = 0; i < pos.rows(); ++i){
        const auto& index = indexes[i];
        for(size_t j = 0; j < index.size(); ++j){
            int jj = index[j];
            if(jj == i){
                // TODO: figure out if i is ok
                continue;
            }
            Connection newConnection;
            newConnection.partner = jj;
            auto candidate = std::lower_bound(cs[i].begin(), cs[i].end(), newConnection, [](const auto& a, const auto& b){return a.partner < b.partner;});
            if(candidate != cs[i].end() && candidate->partner == jj){
                // already exists
                continue;
            }
            added++;
            newConnection.rij = (pos.row(i) - pos.row(jj)).norm();
            cs[i].push_back(std::move(newConnection));
        }
    }
    if(removed != 0 || added != 0){
        BOOST_LOG_TRIVIAL(trace) << "Connections removed: " << removed << ", added: " << added;
    }
}


void ViscoElastic::computeTotalForce(Scene& scene, TimeStep dt){
    m_solver->computeTotalForce(scene, dt);
}

} // Physics

} // d2

} // GooBalls
