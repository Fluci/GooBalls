#pragma once

#include "types.hpp"

#include <memory>

namespace GooBalls {
namespace d2 {

/**
 * Vertices/Triangles: Mesh to render in the scene
 * - vertices: a list of three coordinates, each being one vertex
 * - vertices_color: a list of colors corresponding to each vertex
 * - triangles: a list of three indices pointing into the vertex matrix
 * */
class Mesh {
public:
	// typedefs
	typedef Coordinates2d Coordinates;
	typedef ColorsFloatRGB Colors;
	// Vertices
	const Coordinates& vertices_position() const;
	Coordinates& vertices_position();
	const Colors& vertices_colors() const;
	Colors& vertices_colors();
	// Triangles
	const TriangleList& triangles() const;
	TriangleList& triangles();
private:
	std::shared_ptr<Coordinates> m_vertices;
	Colors m_vertices_colors;
	std::shared_ptr<TriangleList> m_triangles;
};

} // d2

} // GooBalls
