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
	Fluid fluid(particleCoordinates);
	fluid.particles_color().setRandom(10, 3);
	fluid.particles_color().array() += 1.0;
	fluid.particles_color() /= 2.0;
	fluid.particles_radius().setRandom(10, 1);
	fluid.particles_radius().array() += 2.0;
    RenderScene aRenderScene;
	aRenderScene.fluids.push_back(std::move(fluid));
    OpenGLRender render;
    render.render(aRenderScene);
    return 0;
}
