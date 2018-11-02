#include "engine.hpp"

#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

using namespace GooBalls;
using namespace d2;
using namespace Physics;

BOOST_AUTO_TEST_CASE(empty_world) {
    Scene scene;
    Engine physEngine;
    int i = 10;
    while(i--){
        physEngine.advance(scene, 0.1);
    }
}

BOOST_AUTO_TEST_CASE(simple_world) {
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(10, 2));
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(30, 2));
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(10, 3));
    triangles->array() = triangles->unaryExpr([](const VertexIndex x) { return std::abs(x)%30; });
    
    auto fluidPhys = std::make_unique<Fluid>(particleCoordinates);
    fluidPhys->particles_velocity().setRandom(10, 2);
    Scene scene;
    scene.fluid = std::move(fluidPhys);

    Mesh physMesh(verts, triangles);
    scene.meshes.push_back(std::move(physMesh));

    Engine physEngine;

    // init world
    // create scene floor
    b2BodyDef groundBodyDef;
    //groundBodyDef.position.Set(0, -10);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0, 10.0);
    b2Body* groundBody = nullptr;
    // something is wrong with the allocator of box2d
    groundBody = scene.world.CreateBody(&groundBodyDef);
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
    scene.world.SetGravity(b2Vec2(scene.gravity[0], scene.gravity[1]));
    // store pointer to Rigid Body in our Scene-Mesh
    scene.meshes[0].body = scene.world.CreateBody(&bodyDef);
    scene.meshes[0].body->CreateFixture(&fixtureDef);

    int i = 10;
    while(i--){
        physEngine.advance(scene, 0.1);
    }
}
