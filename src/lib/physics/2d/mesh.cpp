#include "mesh.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {

Mesh::Mesh(std::shared_ptr<Mesh::Coordinates> verts, std::shared_ptr<TriangleList> tris) : 
    m_vertices_position(verts),
    m_triangles(tris)
    {
    // empty
}

const Mesh::Coordinates& Mesh::vertices_position() const {
    return *m_vertices_position;
}
Mesh::Coordinates& Mesh::vertices_position() {
    return *m_vertices_position;
}

const TriangleList& Mesh::triangles() const {
    return *m_triangles;
}
TriangleList& Mesh::triangles() {
    return *m_triangles;
}

} // Physics
} // d2
} // GooBalls
