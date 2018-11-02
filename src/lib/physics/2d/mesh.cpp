#include "mesh.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {

Mesh::Mesh() : 
    m_vertices_position_global(new Coordinates()), 
    m_triangles(new TriangleList()){
    // empty
}

Mesh::Mesh(std::shared_ptr<Mesh::Coordinates> verts, std::shared_ptr<TriangleList> tris) : 
    m_vertices_position_global(verts),
    m_triangles(tris)
{
    m_vertices_position_local = *verts;
}

const Mesh::Coordinates& Mesh::vertices_position_global() const {
    return *m_vertices_position_global;
}

Mesh::Coordinates& Mesh::vertices_position_global() {
    return *m_vertices_position_global;
}

const Mesh::Coordinates& Mesh::vertices_position_local() const {
    return m_vertices_position_local;
}
Mesh::Coordinates& Mesh::vertices_position_local() {
    return m_vertices_position_local;
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
