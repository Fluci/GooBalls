#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"
#include "physics/2d/engine.hpp"

using namespace GooBalls;
using namespace d2;

// TODO: this should be refactored somewhere close to the asset loader 
/// Create a random scene as mock data
void createRandomScene(Physics::Scene& physScene, Render::Scene& aRenderScene) {
    // some example data to allow first testing with rendering
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(10, 2));
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(30, 2));
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(10, 3));
    triangles->array() = triangles->unaryExpr([](const VertexIndex x) { return std::abs(x)%30; });

    // create physics data
    auto fluidPhys = std::make_unique<Physics::Fluid>(particleCoordinates);
    fluidPhys->particles_velocity().setRandom(10, 2);
    physScene.fluid = std::move(fluidPhys);

    Physics::Mesh physMesh(verts, triangles);
    physScene.meshes.push_back(std::move(physMesh));


    // create scene floor
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0, -10);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0, 10.0);
    b2Body* groundBody = physScene.world.CreateBody(&groundBodyDef);
    groundBody->CreateFixture(&groundBox, 0.0f);
    // create first dynamic body
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 4.0f);
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
    physScene.world.SetGravity(b2Vec2(physScene.gravity[0], physScene.gravity[1]));
    // store pointer to Rigid Body in our Scene-Mesh
    physScene.meshes[0].body = physScene.world.CreateBody(&bodyDef);
    physScene.meshes[0].body->CreateFixture(&fixtureDef);

    // create data for rendering
    std::unique_ptr<Render::DiskFluid> fluid = std::make_unique<Render::DiskFluid>(particleCoordinates);
    fluid->particles_color().setRandom(10, 3);
    fluid->particles_color().array() += 1.0;
    fluid->particles_color() /= 2.0;
    fluid->particles_radius().setRandom(10, 1);
    fluid->particles_radius().array() += 2.0;
    auto mesh = std::make_unique<Render::Mesh>(verts, triangles);
    mesh->vertices_color().setRandom(30,3);
    mesh->vertices_color().array() += 1.0;
    mesh->vertices_color() /= 2.0;
    aRenderScene.fluids.push_back(std::move(fluid));
    aRenderScene.meshes.push_back(std::move(mesh));

	// test parsing a json
	std::ifstream ifs("test.json");
	Json::Reader reader;
	Json::Value obj;
	reader.parse(ifs, obj);
	std::cout << "Last Name: " << obj["lastname"].asString() << std::endl;
	std::cout << "First Name: " << obj["firstname"] << std::endl;
	std::cout << "Legi: " << obj["legi"].asInt() - 1.5 << std::endl;
}


int main(int argc, char *argv[]){
    std::cout << "Hi there!" << std::endl;
    anotherHello();
    hello();

    Physics::Engine physEngine;
    Physics::Scene physScene;
    Render::Scene aRenderScene;
    createRandomScene(physScene, aRenderScene);
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
