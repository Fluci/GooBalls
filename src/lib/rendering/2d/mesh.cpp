#include "mesh.hpp"
#include <iostream>
#include <memory>

namespace GooBalls {
namespace d2 {
namespace Render {
// constructors

Mesh::Mesh() : 
    m_vertices(new Coordinates()),
    m_triangles(new TriangleList())
{}

Mesh::Mesh(std::shared_ptr<Mesh::Coordinates> verts, std::shared_ptr<TriangleList> triangles) : 
    m_vertices(verts), 
    m_triangles(triangles)
    {
    // empty
}


// accessors

const Mesh::Coordinates& Mesh::vertices_position() const {
    assert(m_vertices.get() != nullptr);
	return *m_vertices;
}

Mesh::Coordinates& Mesh::vertices_position() {
    assert(m_vertices.get() != nullptr);
	return *m_vertices;
}

const Mesh::Colors& Mesh::vertices_color() const {
	return m_vertices_colors;
}

Mesh::Colors& Mesh::vertices_color() {
	return m_vertices_colors;
}

const TriangleList& Mesh::triangles() const {
    assert(m_triangles.get() != nullptr);
	return *m_triangles;
}

TriangleList& Mesh::triangles() {
    assert(m_triangles.get() != nullptr);
	return *m_triangles;
}

// rendering

void Mesh::render() const {
    // mock
    auto& tris = triangles();
    std::cout << "Got " << tris.rows() << " triangles:\n";
    for(int i = 0; i < tris.rows(); ++i){
        std::cout << tris(i,0) << ", " << tris(i, 1) << " " << tris(i, 2) << "\n";
    }
}

} // Render

} // d2

} // GooBalls
