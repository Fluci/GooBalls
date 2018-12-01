#pragma once

#include "types.hpp"
#include <memory>
#include <Box2D/Box2D.h>

namespace GooBalls {
namespace d2 {
namespace Physics {

class Mesh {
public:
    typedef Coordinates2d Coordinates;
    // constructors
    Mesh();
    Mesh(std::shared_ptr<Coordinates> verts, std::shared_ptr<TriangleList> tris);
    // accessors
    const Coordinates& vertices_position_global() const;
    Coordinates& vertices_position_global();
    const Coordinates& vertices_position_local() const;
    Coordinates& vertices_position_local();
    const TriangleList& triangles() const;
    TriangleList& triangles();
    RotationMatrix rotation() const;
    TranslationVector translation() const;
    TranslationVector linear_velocity() const;
    RotationMatrix angular_velocity() const;
    /// takes the rigid body transformation from the box2D object and applies it locally
    void update_from_rigid_body();
    /// Computes everything to make the object ready for the run loop
    /// `h` needs to be known, as the boundary particles are pre-computed as much as possible
    void prepare(Float h);
    const Coordinates2d& particles_position_local() const;
    const Coordinates2d& particles_velocity() const;
    const Coordinates1d& particles_volume() const;
    const Coordinates1d& particles_mass() const;
    Float& particles_velocity_correction_coefficient();
    Float particles_velocity_correction_coefficient() const;
    /**
     * Holds a pointer to its corresponding Box2d object
     * */
    b2Body* body = nullptr;
private:
    /// in world coordinates
    std::shared_ptr<Coordinates> m_vertices_position_global;
    /// in local coordinates
    Coordinates m_vertices_position_local;
    std::shared_ptr<TriangleList> m_triangles;
    /// the local positions need to be set once
    Coordinates m_particles_position_local;
    Coordinates m_particles_velocity;
    Coordinates1d m_particles_volume;
    Coordinates1d m_particles_mass;
    /// For Visco Elastic materials
    Float m_velocity_correction_coefficient = 0.0;
    /// Create fixed particles
    void create_particles(Float h);
    /// Compute weight for particles
    void compute_particles_volume(Float h);
    void compute_particles_mass(Float h);
};

} // Physics
} // d2
} // GooBalls
