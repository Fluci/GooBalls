#include "fluid.hpp"
#include <iostream>
#include "spatial/2d/neighborhood_spatial_hashing.hpp"

#include <boost/log/trivial.hpp>

namespace GooBalls {
namespace d2 {
namespace Physics {

using namespace Spatial;

Fluid::Fluid() : m_particles_position(new Coordinates2d()){
    fluid_neighborhood = std::make_unique<NeighborhoodSpatialHashing>();
    boundary_neighborhood = std::make_unique<NeighborhoodSpatialHashing>();
}

Fluid::Fluid(std::shared_ptr<Coordinates2d> ptr, std::shared_ptr<Coordinates2d> boundary) : m_particles_position(ptr), m_boundary_position(boundary) {
    fluid_neighborhood = std::make_unique<NeighborhoodSpatialHashing>();
    boundary_neighborhood = std::make_unique<NeighborhoodSpatialHashing>();
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

const Coordinates2d& Fluid::particles_external_force() const {
    return m_particles_external_force;
}

Coordinates2d& Fluid::particles_external_force() {
    return m_particles_external_force;
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

Float& Fluid::particles_lower_velocity_correction_limit(){
    return m_lower_velocity_correction_limit;
}

const Float& Fluid::particles_lower_velocity_correction_limit() const {
    return m_lower_velocity_correction_limit;
}

Float& Fluid::particles_weakening_speed(){
    return m_weakening_speed;
}

const Float& Fluid::particles_weakening_speed() const {
    return m_weakening_speed;
}

Float& Fluid::particles_yield_criterion() {
    return m_yield_criterion;
}

const Float& Fluid::particles_yield_criterion() const {
    return m_yield_criterion;
}

Float& Fluid::merge_threshold() {
    return m_merge_threshold;
}

const Float& Fluid::merge_threshold() const {
    return m_merge_threshold;
}

Float& Fluid::split_threshold() {
    return m_split_threshold;
}

const Float& Fluid::split_threshold() const {
    return m_split_threshold;
}

Float& Fluid::boundary_merge_threshold() {
    return m_boundary_merge_threshold;
}

const Float& Fluid::boundary_merge_threshold() const {
    return m_boundary_merge_threshold;
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

const Coordinates1d& Fluid::boundary_psi() const {
    return m_boundary_psi;
}

Coordinates1d& Fluid::boundary_psi() {
    return m_boundary_psi;
}

const Coordinates1d& Fluid::boundary_mass() const {
    return m_boundary_mass;
}
Coordinates1d& Fluid::boundary_mass() {
    return m_boundary_mass;
}

const Coordinates1d& Fluid::boundary_velocity_correction_coefficient() const {
    return m_boundary_velocity_correction_coefficient;
}

Coordinates1d& Fluid::boundary_velocity_correction_coefficient() {
    return m_boundary_velocity_correction_coefficient;
}


Float Fluid::h() const {
    return m_h;
}

void Fluid::h(Float h) {
    m_h = h;
}

Float Fluid::stiffnessConstant() const {
    return m_K;
}
void Fluid::stiffnessConstant(Float k) {
    m_K = k;
}

Float Fluid::rest_density() const{
    return m_rho0;
}

void Fluid::rest_density(Float rho) {
    m_rho0 = rho;
}

Float Fluid::surface_tension() const {
    return m_color_sigma;
}
void Fluid::surface_tension(Float sigma) {
    m_color_sigma = sigma;
}

Float Fluid::fluid_viscosity() const {
    return m_mu;
}
void Fluid::fluid_viscosity(Float m){
    m_mu = m;
}

Float Fluid::boundary_viscosity() const{
    return m_boundary_mu;
}
void Fluid::boundary_viscosity(Float mu_boundary) {
    m_boundary_mu = mu_boundary;
}

Float Fluid::pressure_gamma() const {
    return m_pressure_gamma;
}
void Fluid::pressure_gamma(Float gamma) {
    m_pressure_gamma = gamma;
}

bool Fluid::continuous_merge() const {
    return m_continuous_merge;

}
void Fluid::continuous_merge(bool newV) {
    m_continuous_merge = newV;
}

bool Fluid::continuous_split() const {
    return m_continuous_split;
}
void Fluid::continuous_split(bool newV) {
    m_continuous_split = newV;
}

bool Fluid::sanity_check() const {
    if(particles_position().rows() != particles_velocity().rows()){
        BOOST_LOG_TRIVIAL(error) << "Position and velocity should have same number of rows.";
        return false;
    }
    if(particles_position().rows() != particles_mass().rows()){
        BOOST_LOG_TRIVIAL(error) << "Position and mass should have same number of rows.";
        return false;
    }
    if(particles_mass().minCoeff() <= 0.0){
        BOOST_LOG_TRIVIAL(error) << "Particle mass should be positive.";
        return false;
    }
    return true;
}

} // Physics
} // d2
} // GooBalls
