#include "fluid.hpp"


namespace GooBalls {
namespace d2 {

Fluid::Fluid(std::shared_ptr<Coordinates> ptr) : m_particles_position(ptr){
	// Empty
}

// Fluid

const Fluid::Coordinates& Fluid::particles_position() const {
	assert(m_particles_position.get() != nullptr);
	return *m_particles_position;
}

Fluid::Coordinates& Fluid::particles_position() {
	assert(m_particles_position.get() != nullptr);
	return *m_particles_position;
}

const Fluid::Colors& Fluid::particles_color() const {
	return m_particles_color;
}

Fluid::Colors& Fluid::particles_color() {
	return m_particles_color;
}

const Fluid::Radii& Fluid::particles_radius() const {
	return m_particles_radius;
}

Fluid::Radii& Fluid::particles_radius() {
	return m_particles_radius;
}


} // d2
} // GooBalls
