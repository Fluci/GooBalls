#pragma once
#include "types.hpp"
#include "mesh.hpp"
#include <memory>
#include <vector>

namespace GooBalls {
namespace d2 {

struct Background {
	ColorFloatPrecision r;
	ColorFloatPrecision g;
	ColorFloatPrecision b;
};

/**
 * Coordinate sytem:
 * +x points to the right
 * +y points upwards
 * (0,0) is located at the lower left corner
 */
struct Camera {
	CoordinatePrecision x;
	CoordinatePrecision y;
	CoordinatePrecision width;
	CoordinatePrecision height;
};

/**
 * This class describes a 2D scene that should be rendered.
 *
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
class RenderScene {
public:
	// typedefs
	typedef Coordinates2d Coordinates;
	typedef ColorsFloatRGB Colors;
	typedef Eigen::VectorXd Radii;
	// Generic stuff
	RenderScene(std::shared_ptr<Coordinates>);
	// Particles
	const Coordinates& particles_position() const;
	Coordinates& particles_position();
	const Colors& particles_color() const;
	Colors& particles_color();
	const Radii& particles_radius() const;
	Radii& particles_radius();
	// Meshes
	const std::vector<Mesh>& meshes() const;
	std::vector<Mesh>& meshes();
	// Background
	const Background& background() const;
	Background& background();
	// camera
	const Camera& camera() const;
	Camera& camera();
private:
	std::shared_ptr<Coordinates> m_particles_position;
	Colors m_particles_color;
	Radii m_particles_radius;
	Background m_background;
	Camera m_camera;
	std::vector<Mesh> m_meshes;
};
} // d2
} // GooBalls
