#include "engine.hpp"
#include "box2dmessages.hpp"
#include "visco_elastic.hpp"
#include "generic/is_finite.hpp"
#include "spatial/2d/neighborhood_spatial_hashing.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/log/trivial.hpp>

namespace GooBalls {
namespace d2 {
namespace Physics {
using namespace Spatial;

Engine::Engine() {
    m_fluidSolver = std::make_unique<ViscoElastic>();
}

void Engine::fluidSolver(std::unique_ptr<FluidSolver>&& solver){
    m_fluidSolver = std::move(solver);
}

void Engine::initScene(Scene& scene){
    BOOST_LOG_TRIVIAL(trace) << "Physics Engine: initializing scene";
    for(auto& mesh : scene.meshes){
        mesh.prepare(scene.fluid->h());
    }
    m_fluidSolver->initFluid(scene);
    scene.world.SetGravity(b2Vec2(scene.gravity[0], scene.gravity[1]));
    BOOST_LOG_TRIVIAL(trace) << "Physics Engine: Scene initialized";
}

void Engine::advance(Scene& scene, TimeStep dt) {
    BOOST_LOG_TRIVIAL(trace) << "Physics Engine: advancing scene by " << dt;
    assert(is_finite(scene.fluid->particles_position()));
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
        auto& mass = scene.fluid->boundary_mass();
        auto& velocityCorrection = scene.fluid->boundary_velocity_correction_coefficient();
        boundary.resize(boundaryParticles, Eigen::NoChange);
        volume.resize(boundaryParticles, Eigen::NoChange);
        velocity.resize(boundaryParticles, Eigen::NoChange);
        mass.resize(boundaryParticles, Eigen::NoChange);
        velocityCorrection.resize(boundaryParticles, Eigen::NoChange);
        int s = 0;
        for(int i = 0; i < scene.meshes.size(); ++i){
            auto& mesh = scene.meshes[i];
            const auto& local = mesh.particles_position_local();
            boundary.block(s, 0, local.rows(), 2) = (local * mesh.rotation().transpose()).rowwise() + mesh.translation();//(local * mesh.rotation().transpose()).rowwise() + mesh.translation(); // TODO: check order of arguments
            volume.block(s, 0, local.rows(), 1) = mesh.particles_volume();
            velocity.block(s, 0, local.rows(), 2) = mesh.particles_velocity();
            mass.block(s, 0, local.rows(), 1) = mesh.particles_mass();
            velocityCorrection.block(s, 0, local.rows(), 1).array() = mesh.particles_velocity_correction_coefficient();
            s += local.rows();
            /*
            if(mesh.particles_velocity().maxCoeff() > 50){
                BOOST_LOG_TRIVIAL(info) << "R" << i << ", Max rigid particle speed: " << mesh.particles_velocity().maxCoeff();
            }
            */
        }
    }
    // solve fluid
    m_fluidSolver->advance(scene, dt);
    assert(scene.fluid->boundary_position().rows() == scene.fluid->boundary_force().rows());
    assert(is_finite(scene.fluid->particles_position()));

    if(boundaryParticles > 0){
        int s = 0;
        for(int i = 0; i < scene.meshes.size(); ++i){
            auto& mesh = scene.meshes[i];
            const auto& local = mesh.particles_position_local();
            const auto& force = scene.fluid->boundary_force().block(s, 0, local.rows(), 2);
            const auto& pos = scene.fluid->boundary_position().block(s, 0, local.rows(), 2);
            const auto& ms = scene.fluid->boundary_mass().block(s, 0, local.rows(), 1);
            s += local.rows();

            if(mesh.body->GetMass() == 0.0){
                //BOOST_LOG_TRIVIAL(info) << "Skipping R" << i;
                continue;
            }

            /*
            for(int i = 0; i < part.rows(); ++i){
                mesh.body->ApplyForce(b2Vec2(part(i,0)/pos.rows(), part(i,1)/pos.rows()), b2Vec2(pos(i,0), pos(i,1)), true);
            }
            //*/
            TranslationVector Frigid = force.colwise().mean();
            /*
            auto acc = force.array().colwise()/ms.array();
            if(Frigid.maxCoeff() > 100*1000){
                BOOST_LOG_TRIVIAL(info) << "R" << i << ", Frigid: " << Frigid[0] << ", " << Frigid[1];
            }
            if(acc.maxCoeff() > 1000*1000){
                BOOST_LOG_TRIVIAL(info) << "R" << i << ", Facc: " << acc.maxCoeff();
            }
            */
            mesh.body->ApplyForce(b2Vec2(Frigid[0], Frigid[1]), mesh.body->GetWorldCenter(), true);
            TranslationVector cog = TranslationVector(mesh.body->GetWorldCenter().x, mesh.body->GetWorldCenter().y);
            //cog = pos.colwise.mean();
            Coordinates2d centered = (pos.rowwise() - cog);

            /*
             * Cross product: a x b = (a2 b3 - a3b2, a3b1 - a1b3, a1b2 - a2b1)
             * = (a2 * 0 - 0 * b2, 0 * b1 - a1 * b, a1 b2 - a2 b1)
            */

            Float torque = (centered.col(0)*Frigid[1] - centered.col(1) * Frigid[0]).sum();
            mesh.body->ApplyTorque(torque, true);
        }
    }
    assert(is_finite(scene.fluid->particles_position()));
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
