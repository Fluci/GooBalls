#include "no_sph.hpp"


namespace GooBalls {

namespace d2 {

namespace Physics {

void NoSph::computeTotalForce(Scene& scene, TimeStep) {
    // don't do anything
    if(scene.fluid->particles_position().rows() == 0){
        return;
    }
    scene.fluid->particles_total_force().resize(scene.fluid->particles_position().rows(), Eigen::NoChange);
    scene.fluid->particles_total_force().setZero();
    scene.fluid->particles_density().resize(scene.fluid->particles_position().rows(), Eigen::NoChange);
    scene.fluid->particles_density().setOnes();
    scene.fluid->particles_total_force().col(0) = scene.gravity[0] * scene.fluid->particles_density();
    scene.fluid->particles_total_force().col(1) = scene.gravity[1] * scene.fluid->particles_density();
}

} // Physics

} // d2

} // GooBalls
