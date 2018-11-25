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
    //virtual ~Fluid() = default;

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
    CoordinatePrecision h() const;
    void h(CoordinatePrecision h);

    FloatPrecision K() const;
    void K(FloatPrecision k);

    FloatPrecision rest_density() const;
    void rest_density(FloatPrecision rho0);

    FloatPrecision surface_tension() const;
    void surface_tension(FloatPrecision sigma);

    FloatPrecision fluid_viscosity() const;
    void fluid_viscosity(FloatPrecision mu);

    FloatPrecision boundary_viscosity() const;
    void boundary_viscosity(FloatPrecision mu);

    /// Pressure is commonly computed as p = K/(gamma * rho0) *((rho/rho0)^gamma - 1);
    FloatPrecision pressure_gamma() const;
    void pressure_gamma(FloatPrecision gamma);

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

    FloatPrecision m_K = 10000.0; // gas constant dependent on temperature, good values 1000-100'000
    // rho, density: a value measured in kg/m^3, water: 1000, air: 1.3
    // p, pressure: force per unit area
    // nu, kinematic viscosity: high values: fluid doesn't like to deform, low values: fluid likes deformation
    // often: nu = mu / rho
    // mu, dynamic viscosity coefficient:
    FloatPrecision m_rho0 = 1000.0; // rest density? according to Bridson: environmental pressure?, TODO: get correct base value
    FloatPrecision m_color_sigma = 0.0; // surface tension, TODO: correct value
    FloatPrecision m_mu = .03; // viscosity
    FloatPrecision m_boundary_mu = .03; // viscosity towards wall
    FloatPrecision m_pressure_gamma = 7; // 1..7
};


} // Physics
} // d2
} // GooBalls
