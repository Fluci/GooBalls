#pragma once

#include "types.hpp"
#include "renderable.hpp"
#include <memory>

namespace GooBalls {
namespace d2 {
namespace Render {

/**
 * Vertices/Triangles: Mesh to render in the scene
 * - vertices: a list of three coordinates, each being one vertex
 * - vertices_color: a list of colors corresponding to each vertex
 * - triangles: a list of three indices pointing into the vertex matrix
 * */
class Mesh : public Renderable {
public:
	// typedefs
	typedef Coordinates2d Coordinates;
	typedef ColorsFloatRGB Colors;
    // Constructor
    Mesh();
    Mesh(std::shared_ptr<Coordinates> verts, std::shared_ptr<TriangleList> triangles);
	// Vertices
	const Coordinates& vertices_position() const;
	Coordinates& vertices_position();
	const Colors& vertices_color() const;
	Colors& vertices_color();
	// Triangles
	const TriangleList& triangles() const;
	TriangleList& triangles();
    // rendering
    virtual void render() const;
private:
	std::shared_ptr<Coordinates> m_vertices;
	Colors m_vertices_colors;
	std::shared_ptr<TriangleList> m_triangles;
};

} // Render
} // d2
} // GooBalls
