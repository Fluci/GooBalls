#include "fluid.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {

Fluid::Fluid(std::shared_ptr<Fluid::Coordinates> ptr) : m_particles_position(ptr) {
    // empty
}

const Fluid::Coordinates& Fluid::particles_position() const {
    assert(m_particles_position.get() != nullptr);
    return *m_particles_position;
}
Fluid::Coordinates& Fluid::particles_position(){
    assert(m_particles_position.get() != nullptr);
    return *m_particles_position;
}

const Fluid::Coordinates& Fluid::particles_velocity() const {
    return m_particles_velocity;
}
Fluid::Coordinates& Fluid::particles_velocity() {
    return m_particles_velocity;
}
    
const Fluid::MassList& Fluid::particles_mass() const {
    return m_particles_mass;
}
Fluid::MassList& Fluid::particles_mass() {
    return m_particles_mass;
}

} // Physics
} // d2
} // GooBalls
