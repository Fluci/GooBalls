#include "opengl_render.hpp"
#include <iostream>

namespace GooBalls {
namespace d2 {

void OpenGLRender::render(const RenderScene& scene) {
	// Create nice image and display on screen
	
	// Mock:
    for(const auto& fluid : scene.fluids){
        fluid->render();
    }
}
} // d2
} // GooBalls
