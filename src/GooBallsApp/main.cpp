#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"
#include "physics/2d/engine.hpp"
#include "physics/2d/fluid.hpp"
#include "physics/2d/ssph.hpp"
#include "physics/2d/iisph.hpp"
#include "physics/2d/visco_elastic.hpp"
#include "loader/scene_loader.hpp"

#include "ui/ui.h"
#include "cli_options.hpp"

#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

using namespace GooBalls;
using namespace d2;

// TODO: this should be refactored somewhere close to the asset loader 
/// Create a random scene as mock data
void createRandomScene(Physics::Scene& physScene, Render::Scene& aRenderScene) {
    //physScene.gravity.array() *= 0.0;
    constexpr int PN_X = 10;
    int PN_Y = 20;
    int PN = PN_Y * PN_X;

    // some example data to allow first testing with rendering
    auto particleCoordinates = std::make_shared<Coordinates2d>(Coordinates2d::Random(PN, 2)/2.0);
    for(int i = 0; i < PN_X; ++i){
        for(int j = 0; j < PN_Y; ++j){
            (*particleCoordinates)(i*PN_Y+j, 0) = i;
            (*particleCoordinates)(i*PN_Y+j, 1) = j;
        }
    }
    auto boundaryCoords = std::make_shared<Coordinates2d>();
    (*particleCoordinates) = (*particleCoordinates) * 0.032;
    particleCoordinates->col(0).array() += 0.27;
    particleCoordinates->col(1).array() += 0.7;
    std::cout << "particles: \n";
    for(int i = 0; i < particleCoordinates->rows(); ++i){
        std::cout << (*particleCoordinates)(i,0) << " " << (*particleCoordinates)(i,1) << std::endl;
    }
    int VN = 3*4; // number of verts, multiple of three
    int TN = 2;
    auto verts = std::make_shared<Coordinates2d>(Coordinates2d::Random(VN, 2)*0.5);
    auto triangles = std::make_shared<TriangleList>(TriangleList::Random(TN, 3));
    triangles->array() = triangles->unaryExpr([VN](const VertexIndex x) { return std::abs(x)%VN; });
    double h = 0.05;

    // create physics data
    auto fluidPhys = std::make_unique<Physics::Fluid>(particleCoordinates, boundaryCoords);
    fluidPhys->particles_velocity().setRandom(PN, 2);
    fluidPhys->particles_velocity() *= 0.0;
    fluidPhys->particles_mass().resize(PN);
    // compute the mass from rho*V = m
    fluidPhys->particles_mass().array() = 100;
    fluidPhys->h(h);
    fluidPhys->stiffnessConstant(10000);
    fluidPhys->rest_density(1000.0);
    fluidPhys->surface_tension(0.0);
    fluidPhys->fluid_viscosity(.03);
    fluidPhys->boundary_viscosity(.03);
    fluidPhys->pressure_gamma(7);
    physScene.fluid = std::move(fluidPhys);
    /*
    Physics::Mesh physMesh(verts, triangles);
    physScene.meshes.push_back(std::move(physMesh));


    // create scene floor
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0, -0.8);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(5.0, 0.5);
    b2Body* groundBody = physScene.world.CreateBody(&groundBodyDef);
    groundBody->CreateFixture(&groundBox, 0.0f);
    physScene.meshes[0].body = groundBody;
    //*/
/*
    Physics::Mesh physMesh2(verts, triangles);
    physScene.meshes.push_back(std::move(physMesh2));

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
    // store pointer to Rigid Body in our Scene-Mesh
    physScene.meshes.back().body = physScene.world.CreateBody(&bodyDef);
    //physScene.meshes.back().body->CreateFixture(&fixtureDef);
    physScene.meshes.back().body->CreateFixture(&dynamicBox, 1.0);
        // */
    //physScene.world.SetGravity(b2Vec2(physScene.gravity[0], physScene.gravity[1]));
    // create data for rendering
    std::unique_ptr<Render::DiskFluid> fluid = std::make_unique<Render::DiskFluid>(particleCoordinates, boundaryCoords);
    fluid->particles_color().resize(PN, 3);
    fluid->particles_color().col(0).array() = 0.6;
    fluid->particles_color().col(1).array() = 0.6;
    fluid->particles_color().col(2).array() = 1.0;
    fluid->particles_color() /= 2.0;
    fluid->particles_radius().setOnes(PN, 1);
    fluid->particles_radius().array() += 0.0;
    fluid->particles_radius().array() = 0.008;
    /*
    auto mesh = std::make_unique<Render::Mesh>(verts, triangles);
    mesh->vertices_color().setRandom(VN,3);
    mesh->vertices_color().array() += 1.0;
    mesh->vertices_color() /= 2.0;
    aRenderScene.meshes.push_back(std::move(mesh));
    //*/
    aRenderScene.fluids.push_back(std::move(fluid));

}

namespace op = boost::program_options;
/// Set global log level according to given string, valid values:
/// none, trace, debug, info, warning, error, fatal
/// Unknown values default to info
void setLogLevel(const std::string &level) {
    namespace log = boost::log::trivial;
    if (level == "none") {
        boost::log::core::get()->set_filter(log::severity > log::fatal);
        return;
    }
    auto logFilter = log::info;
    if (level == "trace") {
        logFilter = log::trace;
    } else if (level == "debug") {
        logFilter = log::debug;
    } else if (level == "info") {
        logFilter = log::info;
    } else if (level == "warning") {
        logFilter = log::warning;
    } else if (level == "error") {
        logFilter = log::error;
    } else if (level == "fatal") {
        logFilter = log::fatal;
    } else {
    // empty: default for unknown entries
    }
    boost::log::core::get()->set_filter(log::severity >= logFilter);
}

/// Adds some additional settings to the command line options and parses the passed arguments.
op::variables_map parseCli(int argc, char *argv[],
                           const op::options_description &option_desc) {

    op::positional_options_description pos_desc;
    // allow to shorten "-src <path>" to "<path>"
    pos_desc.add("src", -1);
    op::command_line_parser parser{argc, argv};
    parser.options(option_desc).positional(pos_desc).allow_unregistered();
    op::parsed_options parsed_options = parser.run();
    op::variables_map cli_options;
    op::store(parsed_options, cli_options);
    return cli_options;
}

int main(int argc, char **argv) {
    op::options_description option_desc = cli_options();
    op::variables_map cli_options;

    try {
      cli_options = parseCli(argc, argv, option_desc);
    } catch (const std::string &e) {
      std::cerr << "Exception caught while reading cli arguments: " << e
                << std::endl;
    } catch (const char *e) {
      std::cerr << "Exception caught while reading cli arguments: " << e
                << std::endl;
    } catch (const int e) {
      std::cerr << "Exception caught while reading cli arguments: " << e
                << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Exception caught while reading cli arguments: " << e.what()
                << std::endl;
    }

    if (cli_options.count("help")) {
      std::cout << option_desc;
      return 0;
    }

    setLogLevel(cli_options["log"].as<std::string>());

    std::string scenePath = "../examples/scenes/falling_box.json";
    if(cli_options.count("src")) {
        scenePath = cli_options["src"].as<std::string>();
    }

    Physics::Engine physicsEngine;
    Physics::Scene physicsScene;
    Render::Engine renderEngine;
    Render::Scene renderScene;
    //createRandomScene(physicsScene, renderScene);
    SceneLoader loader;
    if(!loader.loadScene(physicsScene, renderScene, scenePath)){
        return 1;
    }
    //auto tmp = std::make_unique<Physics::SSPH>();
    //auto tmp = std::make_unique<Physics::IISPH>();
    auto tmp = std::make_unique<Physics::ViscoElastic>();
    tmp->considerBoundary(true);
    physicsEngine.fluidSolver(std::move(tmp));
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
