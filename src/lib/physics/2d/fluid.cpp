#include "fluid.hpp"
#include <iostream>

namespace GooBalls {
namespace d2 {
namespace Physics {

Fluid::Fluid() : m_particles_position(new Coordinates2d()){
    // empty
}
Fluid::Fluid(std::shared_ptr<Coordinates2d> ptr) : m_particles_position(ptr) {
    // empty
}

const Coordinates2d& Fluid::particles_position() const {
    assert(m_particles_position.get() != nullptr);
    return *m_particles_position;
}

Coordinates2d& Fluid::particles_position(){
    assert(m_particles_position.get() != nullptr);
    return *m_particles_position;
}

const Coordinates2d& Fluid::particles_velocity() const {
    return m_particles_velocity;
}

Coordinates2d& Fluid::particles_velocity() {
    return m_particles_velocity;
}
    
const Coordinates1d& Fluid::particles_mass() const {
    return m_particles_mass;
}

Coordinates1d& Fluid::particles_mass() {
    return m_particles_mass;
}

const Coordinates1d& Fluid::particles_density() const {
    return m_particles_density;
}

Coordinates1d& Fluid::particles_density(){
    return m_particles_density;
}

const Coordinates2d& Fluid::particles_total_force() const {
    return m_particles_total_force;
}

Coordinates2d& Fluid::particles_total_force() {
    return m_particles_total_force;
}

const Coordinates2d& Fluid::boundary_position() const {
    return m_boundary_position;
}

Coordinates2d& Fluid::boundary_position() {
    return m_boundary_position;
}

const Coordinates2d& Fluid::boundary_velocity() const {
    return m_boundary_velocity;
}

Coordinates2d& Fluid::boundary_velocity() {
    return m_boundary_velocity;
}

const Coordinates1d& Fluid::boundary_volume() const {
    return m_boundary_volume;
}

Coordinates1d& Fluid::boundary_volume() {
    return m_boundary_volume;
}

CoordinatePrecision Fluid::h() const {
    return m_h;
}

void Fluid::h(CoordinatePrecision h) {
    m_h = h;
}

bool Fluid::sanity_check() const {
    if(particles_position().rows() != particles_velocity().rows()){
        std::cerr << "Position and velocity should have same number of rows." << std::endl;
        return false;
    }
    if(particles_position().rows() != particles_mass().rows()){
        std::cerr << "Position and mass should have same number of rows." << std::endl;
        return false;
    }
    return true;
}

} // Physics
} // d2
} // GooBalls
