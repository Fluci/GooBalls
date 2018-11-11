#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"
#include "physics/2d/engine.hpp"
#include "loader/scene_loader.hpp"

#include "ui/ui.h"

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
    fluidPhys->particles_mass().setOnes(10);
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
    fluid->particles_radius().array() += 1.0;
    fluid->particles_radius().array() *= 0.1;
    auto mesh = std::make_unique<Render::Mesh>(verts, triangles);
    mesh->vertices_color().setRandom(30,3);
    mesh->vertices_color().array() += 1.0;
    mesh->vertices_color() /= 2.0;
    aRenderScene.fluids.push_back(std::move(fluid));
    aRenderScene.meshes.push_back(std::move(mesh));

    SceneLoader::loadScene(physScene, aRenderScene, "../examples/scenes/scene0.json");

}


int main(int argc, char **argv) {
    Physics::Engine physicsEngine;
    Physics::Scene physicsScene;
    Render::Engine renderEngine;
    Render::Scene renderScene;
    createRandomScene(physicsScene, renderScene);

    try {
        nanogui::init();

        /* scoped variables */ {
            nanogui::ref<ExampleApplication> app = 
                new ExampleApplication(physicsEngine, renderEngine, physicsScene, renderScene);
            app->drawAll();
            app->setVisible(true);
            nanogui::mainloop();
        }

        nanogui::shutdown();
    } catch (const std::runtime_error &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
        #if defined(_WIN32)
            MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
        #else
            std::cerr << error_msg << endl;
        #endif
        return -1;
    }

    return 0;
}
