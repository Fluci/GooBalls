#include <iostream>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/render_engine.hpp"
#include "rendering/2d/disk_fluid.hpp"

using namespace GooBalls;
using namespace d2;

int main(int argc, char *argv[]){
    std::cout << "Hi there!" << std::endl;
    anotherHello();
    hello();
    // some example data to allow first testing with rendering
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(10, 2));
	auto fluid = std::make_unique<DiskFluid>(particleCoordinates);
	fluid->particles_color().setRandom(10, 3);
	fluid->particles_color().array() += 1.0;
	fluid->particles_color() /= 2.0;
	fluid->particles_radius().setRandom(10, 1);
	fluid->particles_radius().array() += 2.0;
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(30, 2));
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(10, 3));
    triangles->array() = triangles->unaryExpr([](const VertexIndex x) { return std::abs(x)%30; });
    auto mesh = std::make_unique<Mesh>(verts, triangles);
    mesh->vertices_color().setRandom(30,3);
    mesh->vertices_color().array() += 1.0;
    mesh->vertices_color() /= 2.0;
    RenderScene aRenderScene;
	aRenderScene.fluids.push_back(std::move(fluid));
    aRenderScene.meshes.push_back(std::move(mesh));
    RenderEngine render;
    render.render(aRenderScene);
    return 0;
}
