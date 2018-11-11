#pragma once

#include "types.hpp"
#include <memory>

namespace GooBalls {
namespace d2 {
namespace Physics {

class Fluid {
public:
    // constructors
    Fluid();
    Fluid(std::shared_ptr<Coordinates2d> ptr);

    // Field access
    const Coordinates2d& particles_position() const;
    Coordinates2d& particles_position();

    const Coordinates2d& particles_velocity() const;
    Coordinates2d& particles_velocity();
    
    const Coordinates1d& particles_mass() const;
    Coordinates1d& particles_mass();

    const Coordinates1d& particles_density() const;
    Coordinates1d& particles_density();

    const Coordinates2d& particles_total_force() const;
    Coordinates2d& particles_total_force();

    const Coordinates2d& boundary_position() const;
    Coordinates2d& boundary_position();

    const Coordinates2d& boundary_velocity() const;
    Coordinates2d& boundary_velocity();

    const Coordinates1d& boundary_volume() const;
    Coordinates1d& boundary_volume();

    /// Total force that the fluid particles apply to each of the boundary particles
    const Coordinates2d& boundary_force() const;
    /**
     * Checks invariants of the fluid
     * */
    bool sanity_check() const;
    
private:
    std::shared_ptr<Coordinates2d> m_particles_position;
    Coordinates2d m_particles_velocity;
    Coordinates1d m_particles_mass;
    Coordinates1d m_particles_density;
    Coordinates2d m_particles_external_force;
    Coordinates2d m_particles_total_force;
    Coordinates2d m_boundary_position;
    Coordinates2d m_boundary_velocity;
    Coordinates1d m_boundary_volume;
    Coordinates2d m_boundary_force;
};


} // Physics
} // d2
} // GooBalls
