#pragma once
#include "types.hpp"
#include <memory>

namespace GooBalls {
namespace d2 {

struct Background {
	ColorFloatPrecision r;
	ColorFloatPrecision g;
	ColorFloatPrecision b;
};

/**
 * Coordiante sytem:
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
};
} // d2
} // GooBalls
