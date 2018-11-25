#include "fluid.hpp"
#include <iostream>

namespace GooBalls {
namespace d2 {
namespace Physics {

Fluid::Fluid() : m_particles_position(new Coordinates2d()){
    // empty
}
Fluid::Fluid(std::shared_ptr<Coordinates2d> ptr, std::shared_ptr<Coordinates2d> boundary) : m_particles_position(ptr), m_boundary_position(boundary) {
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

const Coordinates1d& Fluid::particles_pressure() const {
    return m_particles_pressure;
}
Coordinates1d& Fluid::particles_pressure() {
    return m_particles_pressure;
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

std::vector<std::vector<Connection>>& Fluid::particles_connectivity() {
    return m_connectivity;
}

const std::vector<std::vector<Connection>>& Fluid::particles_connectivity() const {
    return m_connectivity;
}

Coordinates1d& Fluid::particles_velocity_correction(){
    return m_velocity_correction;
}

const Coordinates1d& Fluid::particles_velocity_correction() const {
    return m_velocity_correction;
}

const Coordinates2d& Fluid::boundary_position() const {
    assert(m_boundary_position.get() != nullptr);
    return *m_boundary_position;
}

Coordinates2d& Fluid::boundary_position() {
    assert(m_boundary_position.get() != nullptr);
    return *m_boundary_position;
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

const Coordinates2d& Fluid::boundary_force() const {
    return m_boundary_force;
}

Coordinates2d& Fluid::boundary_force() {
    return m_boundary_force;
}

CoordinatePrecision Fluid::h() const {
    return m_h;
}

void Fluid::h(CoordinatePrecision h) {
    m_h = h;
}

FloatPrecision Fluid::K() const {
    return m_K;
}
void Fluid::K(FloatPrecision k) {
    m_K = k;
}

FloatPrecision Fluid::rest_density() const{
    return m_rho0;
}

void Fluid::rest_density(FloatPrecision rho) {
    m_rho0 = rho;
}

FloatPrecision Fluid::surface_tension() const {
    return m_color_sigma;
}
void Fluid::surface_tension(FloatPrecision sigma) {
    m_color_sigma = sigma;
}

FloatPrecision Fluid::fluid_viscosity() const {
    return m_mu;
}
void Fluid::fluid_viscosity(FloatPrecision m){
    m_mu = m;
}

FloatPrecision Fluid::boundary_viscosity() const{
    return m_boundary_mu;
}
void Fluid::boundary_viscosity(FloatPrecision mu_boundary) {
    m_boundary_mu = mu_boundary;
}

FloatPrecision Fluid::pressure_gamma() const {
    return m_pressure_gamma;
}
void Fluid::pressure_gamma(FloatPrecision gamma) {
    m_pressure_gamma = gamma;
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
