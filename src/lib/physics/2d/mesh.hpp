#pragma once

#include "types.hpp"
#include <memory>

namespace GooBalls {
namespace d2 {
namespace Physics {

class Mesh {
public:
    typedef Coordinates2d Coordinates;
    // constructors
    Mesh(std::shared_ptr<Coordinates> verts, std::shared_ptr<TriangleList> tris);
    // accessors
    const Coordinates& vertices_position() const;
    Coordinates& vertices_position();
    const TriangleList& triangles() const;
    TriangleList& triangles();
private:
    std::shared_ptr<Coordinates> m_vertices_position;
    std::shared_ptr<TriangleList> m_triangles;
};

} // Physics
} // d2
} // GooBalls
