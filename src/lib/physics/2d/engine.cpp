#include "engine.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace GooBalls {
namespace d2 {
namespace Physics {

void Engine::advance(Scene& scene, TimeStep dt) {
    scene.world.Step(dt, velocity_iterations, position_iterations);
    return;
    // The rigid bodies moved: use their transform to compute the local meshes
    for(auto& mesh : scene.meshes){
        Eigen::Matrix<FloatPrecision, 1, 2> dx(1,2);
        dx[0] = mesh.body->GetPosition().x;
        dx[1] = mesh.body->GetPosition().y;
        FloatPrecision radianAngle = mesh.body->GetAngle();
        // eigen convention: counter-clockwise rotation in radians
        // box2d convention: not clear, only hint is in chapter 8.6 Revolute Joint of the manual -> test
        // TODO: test rotation direction of Box2d
        Eigen::Rotation2D<FloatPrecision> rot(radianAngle);
        Eigen::Matrix<FloatPrecision, 2, 2> rotM = rot.toRotationMatrix();
        const auto& loc = mesh.vertices_position_local();
        auto& glob = mesh.vertices_position_global();
        // TODO: it is not clear, if rotM needs to be transposed here -> test
        rotM.transpose();
        glob = loc * rotM;
        glob = glob.rowwise() + dx;
    }
    if(scene.fluid.get() != nullptr){
        scene.fluid->particles_position() += scene.fluid->particles_velocity()*dt;
    }
}

} // Physics
} // d2
} // GooBalls
