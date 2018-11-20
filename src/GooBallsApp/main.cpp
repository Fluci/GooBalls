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
    int PN = 100;
    //physScene.gravity.array() *= 0.01;

    // some example data to allow first testing with rendering
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(PN, 2)/2.0);
    for(int i = 0; i < 10; ++i){
        for(int j = 0; j < 10; ++j){
            (*particleCoordinates)(i*10+j, 0) = i;
            (*particleCoordinates)(i*10+j, 1) = j;
        }
    }
    auto boundaryCoords = std::make_shared<Coordinates2d>();
    (*particleCoordinates) = (*particleCoordinates) * 0.05;
    std::cout << "particles: \n";
    for(int i = 0; i < particleCoordinates->rows(); ++i){
        std::cout << (*particleCoordinates)(i,0) << " " << (*particleCoordinates)(i,1) << std::endl;
    }
    int VN = 3*4; // number of verts, multiple of three
    int TN = 2;
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(VN, 2)*0.5);
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(TN, 3));
    triangles->array() = triangles->unaryExpr([VN](const VertexIndex x) { return std::abs(x)%VN; });


    // create physics data
    auto fluidPhys = std::make_unique<Physics::Fluid>(particleCoordinates, boundaryCoords);
    fluidPhys->particles_velocity().setRandom(PN, 2);
    fluidPhys->particles_velocity() *= 0.0;
    fluidPhys->particles_mass().resize(PN);
    fluidPhys->particles_mass().array() = 65.0;
    fluidPhys->h(0.2);
    physScene.fluid = std::move(fluidPhys);
    /*
    Physics::Mesh physMesh(verts, triangles);
    physScene.meshes.push_back(std::move(physMesh));
    Physics::Mesh physMesh2(verts, triangles);
    physScene.meshes.push_back(std::move(physMesh2));


    // create scene floor
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0, -0.8);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(5.0, 0.5);
    b2Body* groundBody = physScene.world.CreateBody(&groundBodyDef);
    groundBody->CreateFixture(&groundBox, 0.0f);
    physScene.meshes[0].body = groundBody;

    // create first dynamic body
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 1.0f);
    // create collision object (a polygon shape) -> bounding box
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(0.2f, 0.2f);
    // create a fixture: 
    // this defines physical properties of the collision shape
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;
    // attach to body
    physScene.world.SetGravity(b2Vec2(physScene.gravity[0], physScene.gravity[1]));
    // store pointer to Rigid Body in our Scene-Mesh
    physScene.meshes[1].body = physScene.world.CreateBody(&bodyDef);
    physScene.meshes[1].body->CreateFixture(&fixtureDef);
    // */
    // create data for rendering
    std::unique_ptr<Render::DiskFluid> fluid = std::make_unique<Render::DiskFluid>(particleCoordinates, boundaryCoords);
    fluid->particles_color().resize(PN, 3);
    fluid->particles_color().col(0).array() = 0.6;
    fluid->particles_color().col(1).array() = 0.6;
    fluid->particles_color().col(2).array() = 1.0;
    fluid->particles_color() /= 2.0;
    fluid->particles_radius().setOnes(PN, 1);
    fluid->particles_radius().array() += 0.0;
    fluid->particles_radius().array() *= 0.03;
    /*
    auto mesh = std::make_unique<Render::Mesh>(verts, triangles);
    mesh->vertices_color().setRandom(VN,3);
    mesh->vertices_color().array() += 1.0;
    mesh->vertices_color() /= 2.0;
    aRenderScene.meshes.push_back(std::move(mesh));
    //*/
    aRenderScene.fluids.push_back(std::move(fluid));

}


int main(int argc, char **argv) {
    Physics::Engine physicsEngine;
    Physics::Scene physicsScene;
    Render::Engine renderEngine;
    Render::Scene renderScene;
    createRandomScene(physicsScene, renderScene);
    SceneLoader::loadScene(physicsScene, renderScene, "../examples/scenes/scene0.json");
    physicsEngine.initScene(physicsScene);
    std::cout << "Starting gui" << std::endl;
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
