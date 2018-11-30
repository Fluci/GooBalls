#pragma once

#include "types.hpp"
#include "spatial/2d/neighborhood.hpp"
#include <memory>
#include <vector>

namespace GooBalls {
namespace d2 {
namespace Physics {

using namespace Spatial;

/// The particles of a fluid can be pairwise connected.
/// This struct describes that connection.
struct Connection {
    /// index of other particle
    int partner;
    /// ||r_j - r_i||
    Float rij;
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

    const Coordinates1d& particles_pressure() const;
    Coordinates1d& particles_pressure();

    const Coordinates1d& particles_density() const;
    Coordinates1d& particles_density();

    const Coordinates2d& particles_external_force() const;
    Coordinates2d& particles_external_force();

    const Coordinates2d& particles_total_force() const;
    Coordinates2d& particles_total_force();

    std::vector<std::vector<Connection>>& particles_connectivity();
    const std::vector<std::vector<Connection>>& particles_connectivity() const;

    Coordinates1d& particles_velocity_correction();
    const Coordinates1d& particles_velocity_correction() const;

    Float& particles_lower_velocity_correction_limit();
    const Float& particles_lower_velocity_correction_limit() const;

    Float& particles_weakening_speed();
    const Float& particles_weakening_speed() const;

    Float& particles_yield_criterion();
    const Float& particles_yield_criterion() const;

    Float& merge_threshold();
    const Float& merge_threshold() const;

    Float& split_threshold();
    const Float& split_threshold() const;

    const Coordinates2d& boundary_position() const;
    Coordinates2d& boundary_position();

    const Coordinates2d& boundary_velocity() const;
    Coordinates2d& boundary_velocity();

    const Coordinates1d& boundary_volume() const;
    Coordinates1d& boundary_volume();

    const Coordinates1d& boundary_psi() const;
    Coordinates1d& boundary_psi();

    /// Total force that the fluid particles apply to each of the boundary particles
    const Coordinates2d& boundary_force() const;
    Coordinates2d& boundary_force();


    /**
     * Checks invariants of the fluid
     * */
    bool sanity_check() const;

    /// Sphere of influence for each particle
    Float h() const;
    void h(Float h);

    /// in papers often K, the higher, the less the fluid likes to be squished
    Float stiffnessConstant() const;
    void stiffnessConstant(Float k);

    Float rest_density() const;
    void rest_density(Float rho0);

    Float surface_tension() const;
    void surface_tension(Float sigma);

    Float fluid_viscosity() const;
    void fluid_viscosity(Float mu);

    Float boundary_viscosity() const;
    void boundary_viscosity(Float mu);

    /// Pressure is commonly computed as p = rho0*K/gamma *((rho/rho0)^gamma - 1);
    Float pressure_gamma() const;
    void pressure_gamma(Float gamma);

    std::unique_ptr<Neighborhood> fluid_neighborhood;
    std::unique_ptr<Neighborhood> boundary_neighborhood;

private:
    std::shared_ptr<Coordinates2d> m_particles_position;
    Coordinates2d m_particles_velocity;
    Coordinates1d m_particles_mass;
    Coordinates1d m_particles_density;
    Coordinates2d m_particles_external_force;
    Coordinates2d m_particles_total_force;
    Coordinates1d m_particles_pressure;
    std::shared_ptr<Coordinates2d> m_boundary_position;
    Coordinates2d m_boundary_velocity;
    Coordinates1d m_boundary_volume;
    Coordinates2d m_boundary_force;
    Coordinates1d m_boundary_psi;
    Float m_h = 0.05;
    std::vector<std::vector<Connection>> m_connectivity;
    /// c_i: velocity correction coefficient, the higher, the tougher the material
    Coordinates1d m_velocity_correction;
    /// l_i: lower bound for c_i
    Float m_lower_velocity_correction_limit = 0.0;
    Float m_weakening_speed = 0.0;
    /// Roughly: if a paritle pair is further than h*gamma, it get's weakened
    Float m_yield_criterion = 1.5;
    Float m_merge_threshold = 1.0;
    Float m_split_threshold = 3.0;

    Float m_K = 10000.0; // gas constant dependent on temperature, good values 1000-100'000
    // rho, density: a value measured in kg/m^3, water: 1000, air: 1.3
    // p, pressure: force per unit area
    // nu, kinematic viscosity: high values: fluid doesn't like to deform, low values: fluid likes deformation
    // often: nu = mu / rho
    // mu, dynamic viscosity coefficient:
    Float m_rho0 = 1000.0; // rest density? according to Bridson: environmental pressure?, TODO: get correct base value
    Float m_color_sigma = 0.0; // surface tension, TODO: correct value
    Float m_mu = .03; // viscosity
    Float m_boundary_mu = .03; // viscosity towards wall
    Float m_pressure_gamma = 7; // 1..7
};


} // Physics
} // d2
} // GooBalls
