#include <iostream>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/opengl_render.hpp"

using namespace GooBalls;
using namespace d2;

int main(int argc, char *argv[]){
    std::cout << "Hi there!" << std::endl;
    anotherHello();
    hello();
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(10, 2));
    RenderScene aRenderScene(particleCoordinates);
	aRenderScene.particles_color().setRandom(10, 3);
	aRenderScene.particles_color().array() += 1.0;
	aRenderScene.particles_color() /= 2.0;
	aRenderScene.particles_radius().setRandom(10, 1);
	aRenderScene.particles_radius().array() += 2.0;
    OpenGLRender render;
    render.render(aRenderScene);
    return 0;
}
