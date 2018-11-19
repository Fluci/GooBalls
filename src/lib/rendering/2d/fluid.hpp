#pragma once

#include "types.hpp"
#include "renderable.hpp"
#include <memory>

namespace GooBalls {
namespace d2 {
namespace Render {

/**
 * For all attributes, accessors are provided.
 *
 * Following attributes are included:
 *
 * Particles: these represent the fluid in the scene
 * - position: list of 2d coordinates
 * - color: list of colors
 * - radius: list of point radii to use when rendering
 *
 *
 * */

class Fluid : public Renderable {
public:
// typedefs
    typedef Coordinates2d Coordinates;
    typedef ColorsFloatRGB Colors;
    typedef Eigen::VectorXf Radii;
    // constructor
    Fluid();
    Fluid(std::shared_ptr<Coordinates> ptr, std::shared_ptr<Coordinates> boundary = nullptr);
    // particles
    const Coordinates& particles_position() const;
    Coordinates& particles_position();
    const Colors& particles_color() const;
    Colors& particles_color();
    const Radii& particles_radius() const;
    Radii& particles_radius();

    const Coordinates& boundary_position() const;
    Coordinates& boundary_position();
    const Colors& boundary_color() const;
    Colors& boundary_color();
    const Radii& boundary_radius() const;
    Radii& boundary_radius();
private:
    std::shared_ptr<Coordinates> m_particles_position;
    std::shared_ptr<Coordinates> m_boundary_position;
    Colors m_particles_color;
    Radii m_particles_radius;
    Colors m_boundary_color;
    Radii m_boundary_radius;
};

} // Render

} // d2

} // GooBalls
