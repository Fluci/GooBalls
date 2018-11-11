#include "engine.hpp"
#include "box2dmessages.hpp"
#include "ssph.hpp"

#include <Eigen/Core>

namespace GooBalls {
namespace d2 {
namespace Physics {

void Engine::initScene(Scene& scene){
    for(auto& mesh : scene.meshes){
        mesh.prepare(m_fluidSolver.h());
    }
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
    m_fluidSolver.advance(scene, dt);

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
