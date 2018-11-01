#include <iostream>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"
#include "physics/2d/engine.hpp"

using namespace GooBalls;
using namespace d2;

// TODO: this should be refactored somewhere close to the asset loader 
// s.t. it transforms meshes into rigid bodies
/// Set up of the box2d world object
void initWorld(Physics::Scene& scene) {
    scene.world = b2World(b2Vec2(scene.gravity[0], scene.gravity[1]));
    // create scene floor
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0, -10);
    b2Body* groundBody = scene.world.CreateBody(&groundBodyDef);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0, 10.0);
    groundBody->CreateFixture(&groundBox, 0.0f);
    // create first dynamic body
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 4.0f);
    if(scene.meshes.empty()){
        return;
    }
    // store pointer to Rigid Body in our Scene-Mesh
    scene.meshes[0].body = scene.world.CreateBody(&bodyDef);
    // create collision object (a polygon shape) -> bounding box
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);
    // create a fixture: 
    // this defines physical properties of the collision shape
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;
    // attach to body
    scene.meshes[0].body->CreateFixture(&fixtureDef);
}


int main(int argc, char *argv[]){
    std::cout << "Hi there!" << std::endl;
    anotherHello();
    hello();
    // some example data to allow first testing with rendering
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(10, 2));
    auto particleCoordinates2 = std::make_shared<Coordinates2d>(Coordinates2d::Random(10, 2));
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(30, 2));
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(10, 3));
    triangles->array() = triangles->unaryExpr([](const VertexIndex x) { return std::abs(x)%30; });
    // create physics data
    auto fluidPhys = std::make_unique<Physics::Fluid>(particleCoordinates);
    fluidPhys->particles_velocity().setRandom(10, 2);
    Physics::Scene physScene;
    physScene.fluid = std::move(fluidPhys);

    Physics::Mesh physMesh(verts, triangles);
    physScene.meshes.push_back(std::move(physMesh));

    Physics::Engine physEngine;
    initWorld(physScene);
    
    // create data for rendering
    assert(particleCoordinates.get() != nullptr);
    std::unique_ptr<Render::DiskFluid> fluid = std::make_unique<Render::DiskFluid>(particleCoordinates2);
    //Render::DiskFluid* ff = new Render::DiskFluid(particleCoordinates);
    //Render::DiskFluid ff(particleCoordinates);
    //std::unique_ptr<Render::DiskFluid> fluid(ff);
    fluid->particles_color().setRandom(10, 3);
    fluid->particles_color().array() += 1.0;
    fluid->particles_color() /= 2.0;
    fluid->particles_radius().setRandom(10, 1);
    fluid->particles_radius().array() += 2.0;
    auto mesh = std::make_unique<Render::Mesh>(verts, triangles);
    mesh->vertices_color().setRandom(30,3);
    mesh->vertices_color().array() += 1.0;
    mesh->vertices_color() /= 2.0;
    Render::Scene aRenderScene;
    aRenderScene.fluids.push_back(std::move(fluid));
    aRenderScene.meshes.push_back(std::move(mesh));
    Render::Engine render;
    // DEMO
    // show what we can
    std::cout << "first frame: " << std::endl;
    render.render(aRenderScene);
    // do one step
    // as both scene objects share the underlying particle positions, 
    // we don't need to copy anything for the render engine
    physEngine.advance(physScene, 0.1);
    std::cout << "second frame: " << std::endl;
    render.render(aRenderScene);
    return 0;
}
