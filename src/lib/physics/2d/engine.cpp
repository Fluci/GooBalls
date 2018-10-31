#include "engine.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {

void Engine::advance(Scene& scene, TimeStep dt) {
    scene.fluid->particles_position() += scene.fluid->particles_velocity()*dt;
}

} // Physics
} // d2
} // GooBalls
