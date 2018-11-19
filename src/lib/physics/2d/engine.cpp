#include "engine.hpp"
#include "box2dmessages.hpp"
#include "ssph.hpp"
#include "visco_elastic.hpp"
#include "no_sph.hpp"
#include "spatial/2d/neighborhood_spatial_hashing.hpp"

#include <Eigen/Core>

#include <boost/log/trivial.hpp>

namespace GooBalls {
namespace d2 {
namespace Physics {
using namespace Spatial;

Engine::Engine() {
    //m_fluidSolver = std::make_unique<SSPH>();
    //m_fluidSolver = std::make_unique<ViscoElastic>();
    m_fluidSolver = std::make_unique<NoSph>();
}

void Engine::initScene(Scene& scene){
    for(auto& mesh : scene.meshes){
        mesh.prepare(scene.fluid->h());
    }
    const auto& pos = scene.fluid->particles_position();
    if(pos.rows() != scene.fluid->particles_velocity().rows()){
        // by default, give the particles no speed
        BOOST_LOG_TRIVIAL(warning) << "No proper fluid particle velocity set, setting to zero.";
        scene.fluid->particles_velocity().setOnes(pos.rows(), Eigen::NoChange);
    }
    if(pos.rows() != scene.fluid->particles_mass().rows()){
        BOOST_LOG_TRIVIAL(warning) << "No proper fluid particle mass set, setting to zero.";
        scene.fluid->particles_mass().setZero(pos.rows(), Eigen::NoChange);
    }
    auto& conn = scene.fluid->particles_connectivity();
    if(pos.rows() != conn.size()){
        BOOST_LOG_TRIVIAL(warning) << "No proper fluid particle connectivity set, setting to 1.5*h.";
        NeighborhoodSpatialHashing neigh;
        neigh.inRange(pos, scene.fluid->h()*1.5);
        conn.resize(pos.rows());
        for(int i = 0; i < pos.rows(); ++i){
            const auto& index = neigh.indexes()[i];
            conn[i].resize(index.size());
            for(size_t j = 0; j < index.size(); ++j){
                int jj = index[j];
                conn[i][j].partner = jj;
                conn[i][j].rij = (pos.row(j) - pos.row(i)).norm();
            }
        }
    }
    if(pos.rows() != scene.fluid->particles_velocity_correction().rows()){
        BOOST_LOG_TRIVIAL(warning) << "No proper fluid particle velocity correction coefficients set, setting to 1.";
        scene.fluid->particles_velocity_correction().setOnes(pos.rows(), Eigen::NoChange);
    }
    // adjusted later down the road, just making sure the width is ok
    scene.fluid->particles_density().resize(1, Eigen::NoChange);
    scene.fluid->particles_total_force().resize(1, Eigen::NoChange);
}

void Engine::advance(Scene& scene, TimeStep dt) {
    scene.world.Step(dt, velocity_iterations, position_iterations);
    // The rigid bodies moved: use their transform to compute the local meshes
    int boundaryParticles = 0;
    for(auto& mesh : scene.meshes){
        mesh.update_from_rigid_body();
        boundaryParticles += mesh.particles_position_local().rows();
    }
    if(boundaryParticles > 0){
        // transfer rigid body particles to fluid
        auto& boundary = scene.fluid->boundary_position();
        auto& volume = scene.fluid->boundary_volume();
        auto& velocity = scene.fluid->boundary_velocity();
        boundary.resize(boundaryParticles, Eigen::NoChange);
        volume.resize(boundaryParticles, Eigen::NoChange);
        velocity.resize(boundaryParticles, Eigen::NoChange);
        int s = 0;
        for(auto& mesh : scene.meshes){
            const auto& local = mesh.particles_position_local();
            boundary.block(s, 0, local.rows(), 2) = (local * mesh.rotation().transpose()).rowwise() + mesh.translation(); // TODO: check order of arguments
            volume.block(s, 0, local.rows(), 1) = mesh.particles_volume();
            velocity.block(s, 0, local.rows(), 2) = mesh.particles_velocity();
            s += local.rows();
        }
    }
    // solve fluid
    m_fluidSolver->advance(scene, dt);

    // TODO: transfer forces of boundary particles back
}

void Engine::BeginContact(b2Contact* contact) {
    auto msg = std::shared_ptr<Box2DBeginContactMessage>();
    msg->contact = contact;
    sendMessage(msg);
}
void Engine::EndContact(b2Contact* contact){
    auto msg = std::shared_ptr<Box2DEndContactMessage>();
    msg->contact = contact;
    sendMessage(msg);
}
void Engine::PreSolve(b2Contact* contact, const b2Manifold* oldManifold){
    auto msg = std::shared_ptr<Box2DPreSolveMessage>();
    msg->contact = contact;
    msg->oldManifold = oldManifold;
    sendMessage(msg);
}
void Engine::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse){
    auto msg = std::shared_ptr<Box2DPostSolveMessage>();
    msg->contact = contact;
    msg->impulse = impulse;
}


} // Physics
} // d2
} // GooBalls
