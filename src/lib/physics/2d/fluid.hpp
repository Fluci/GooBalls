#pragma once

#include "types.hpp"
#include <memory>
#include <vector>

namespace GooBalls {
namespace d2 {
namespace Physics {

/// The particles of a fluid can be pairwise connected.
/// This struct describes that connection.
struct Connection {
    /// index of other particle
    int partner;
    /// ||r_j - r_i||
    CoordinatePrecision rij;
};


/// Describes a fluid for the physics engine
class Fluid {
public:
    // constructors
    Fluid();
    Fluid(std::shared_ptr<Coordinates2d> ptr, std::shared_ptr<Coordinates2d> boundary = nullptr);

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

    std::vector<std::vector<Connection>>& particles_connectivity();
    const std::vector<std::vector<Connection>>& particles_connectivity() const;

    Coordinates1d& particles_velocity_correction();
    const Coordinates1d& particles_velocity_correction() const;

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

    /// Sphere of influence for each particle
    virtual CoordinatePrecision h() const;
    virtual void h(CoordinatePrecision h);
private:
    std::shared_ptr<Coordinates2d> m_particles_position;
    Coordinates2d m_particles_velocity;
    Coordinates1d m_particles_mass;
    Coordinates1d m_particles_density;
    Coordinates2d m_particles_external_force;
    Coordinates2d m_particles_total_force;
    std::shared_ptr<Coordinates2d> m_boundary_position;
    Coordinates2d m_boundary_velocity;
    Coordinates1d m_boundary_volume;
    Coordinates2d m_boundary_force;
    CoordinatePrecision m_h = 0.05;
    std::vector<std::vector<Connection>> m_connectivity;
    /// c_i: velocity correction coefficient
    Coordinates1d m_velocity_correction;
};


} // Physics
} // d2
} // GooBalls
