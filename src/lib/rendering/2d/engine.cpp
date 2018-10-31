#include "engine.hpp"

namespace GooBalls {
namespace d2 {
namespace Render {

void Engine::render(const Scene& scene) {
    // Create nice image and display on screen
    
    // Mock:
    for(const auto& fluid : scene.fluids){
        fluid->render();
    }   
    for(const auto& mesh : scene.meshes){
        mesh->render();
    }   
}
} // Render
} // d2
} // GooBalls
