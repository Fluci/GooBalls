#pragma once
#include "types.hpp"
#include <memory>

namespace GooBalls {
namespace d2 {

class RenderScene {
public:
	typedef Coordinates2d Coordinates;
	typedef ColorsFloatRGB Colors;
	typedef Eigen::VectorXd Radii;
	RenderScene(std::shared_ptr<Coordinates>);
	const Coordinates& particles_position() const;
	Coordinates& particles_position();
	const Colors& particles_color() const;
	Colors& particles_color();
	const Radii& particles_radius() const;
	Radii& particles_radius();
private:
	std::shared_ptr<Coordinates> m_particles_position;
	Colors m_particles_color;
	Radii m_particles_radius;
};
} // d2
} // GooBalls
