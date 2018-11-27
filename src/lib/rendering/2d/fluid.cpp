#include "fluid.hpp"


namespace GooBalls {
namespace d2 {
namespace Render {

Fluid::Fluid() : m_particles_position(new Coordinates()){
    // Empty
}
Fluid::Fluid(std::shared_ptr<Coordinates> ptr,  std::shared_ptr<Coordinates> boundary) : m_particles_position(ptr), m_boundary_position(boundary){
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

const Fluid::Coordinates& Fluid::boundary_position() const {
    assert(m_boundary_position.get() != nullptr);
    return *m_boundary_position;
}

Fluid::Coordinates& Fluid::boundary_position() {
    assert(m_boundary_position.get() != nullptr);
    return *m_boundary_position;
}

const Fluid::Colors& Fluid::boundary_color() const {
    return m_boundary_color;
}

Fluid::Colors& Fluid::boundary_color() {
    return m_boundary_color;
}

const Fluid::Radii& Fluid::boundary_radius() const {
    return m_boundary_radius;
}

Fluid::Radii& Fluid::boundary_radius() {
    return m_boundary_radius;
}


} // Render
} // d2
} // GooBalls
