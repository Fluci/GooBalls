#include "render_engine.hpp"
#include <iostream>

namespace GooBalls {
namespace d2 {

void RenderEngine::render(const RenderScene& scene) {
    // Create nice image and display on screen
    
    // Mock:
    for(const auto& fluid : scene.fluids){
        fluid->render();
    }   
    for(const auto& mesh : scene.meshes){
        mesh->render();
    }   
}
} // d2
} // GooBalls
