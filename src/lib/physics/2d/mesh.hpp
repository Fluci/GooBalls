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
    /**
     * Holds a pointer to it's corresponding Box2d object
     * */
    b2Body* body = nullptr;
private:
    /// in world coordinates
    std::shared_ptr<Coordinates> m_vertices_position_global;
    /// in local coordinates
    Coordinates m_vertices_position_local;
    std::shared_ptr<TriangleList> m_triangles;
};

} // Physics
} // d2
} // GooBalls
