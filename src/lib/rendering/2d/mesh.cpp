#include "mesh.hpp"

namespace GooBalls {
namespace d2 {

const Mesh::Coordinates& Mesh::vertices_position() const {
	return *m_vertices;
}

Mesh::Coordinates& Mesh::vertices_position() {
	return *m_vertices;
}

const Mesh::Colors& Mesh::vertices_colors() const {
	return m_vertices_colors;
}

Mesh::Colors& Mesh::vertices_colors() {
	return m_vertices_colors;
}

const TriangleList& Mesh::triangles() const {
	return *m_triangles;
}

TriangleList& Mesh::triangles() {
	return *m_triangles;
}

} // d2

} // GooBalls
