#include "opengl_render.hpp"
#include <iostream>

namespace GooBalls {
namespace d2 {

void OpenGLRender::render(const RenderScene& scene) {
	// Create nice image and display on screen
	
	// Mock:
	auto& partCoords = scene.particles_position();
	auto& partColors = scene.particles_color();
	std::cout << "Got " << partCoords.rows() << " particles:\n";
	for(int i = 0; i < partCoords.rows(); ++i){
		std::cout << "xy: " << partCoords(i, 0) << ", " << partCoords(i, 1) << ", ";
		std::cout << "rgb: " << partColors(i, 0) << ", " << partColors(i, 1) << ", " << partColors(i, 2) << ", ";
		std::cout << "r: " << scene.particles_radius()[i] << "\n";
	}
}
} // d2
} // GooBalls
