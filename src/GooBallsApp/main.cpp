#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

#include "placeholder.hpp"
#include "generic/hello.hpp"
#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"
#include "physics/2d/engine.hpp"
#include "physics/2d/fluid.hpp"
#include "physics/2d/fluid_solver.hpp"
#include "physics/2d/ssph.hpp"
#include "physics/2d/iisph.hpp"
#include "physics/2d/visco_elastic.hpp"
#include "loader/scene_loader.hpp"
#include "styler/no_style.hpp"
#include "styler/pressure_density.hpp"

#include "ui/ui.h"
#include "cli_options.hpp"

#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <ctime>
#include <omp.h>

using namespace GooBalls;
using namespace d2;

namespace op = boost::program_options;
/// Set global log level according to given string, valid values:
/// none, trace, debug, info, warning, error, fatal
/// Unknown values default to info
void setLogLevel(const std::string &level) {
    // try to avoid hyperthreading
    int threads = omp_get_max_threads()/2;
    omp_set_num_threads(std::max(1, threads));
    BOOST_LOG_TRIVIAL(debug) << "OMP threads: " << omp_get_max_threads();
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

    std::string scenePath = "../examples/scenes/falling_boxes.json";
    if(cli_options.count("src")) {
        scenePath = cli_options["src"].as<std::string>();
    }

    Physics::Engine physicsEngine;
    Physics::Scene physicsScene;
    Render::Engine renderEngine;
    Render::Scene renderScene;
    physicsScene.room.left_wall = -1;
    physicsScene.room.right_wall = 1;
    physicsScene.room.floor = -1;
    //createRandomScene(physicsScene, renderScene);
    SceneLoader loader;
    // TODO: add fluid solver to scene file
    if(!loader.loadScene(physicsScene, renderScene, scenePath)){
        return 1;
    }
    std::unique_ptr<Physics::FluidSolver> solver;
    auto desiredSolver = cli_options["fluid-solver"].as<std::string>();
    Float dt = cli_options["time-step"].as<double>();
    if(desiredSolver == "ssph"){
        solver = std::make_unique<Physics::SSPH>();
    } else if (desiredSolver == "viscoElastic") {
        solver = std::make_unique<Physics::ViscoElastic>();
    } else if (desiredSolver == "iisph") {
        solver = std::make_unique<Physics::IISPH>();
        dt = 0.001;
    }
    physicsEngine.fluidSolver(std::move(solver));
    physicsEngine.initScene(physicsScene);

    std::unique_ptr<Styler> styler;
    auto desiredStyler = cli_options["styler"].as<std::string>();
    if(desiredStyler == "pressureDensity"){
        styler = std::make_unique<PressureDensity>();
    } else {
        styler = std::make_unique<NoStyle>();
    }
    BOOST_LOG_TRIVIAL(debug) << "Starting gui";
    try {
        BOOST_LOG_TRIVIAL(debug) << "Initializing nanogui";
        nanogui::init();
        /* scoped variables */ {
            BOOST_LOG_TRIVIAL(debug) << "Constructing nanogui application";
            nanogui::ref<ExampleApplication> app =
                new ExampleApplication(physicsEngine, renderEngine, physicsScene, renderScene);
            app->styler = std::move(styler);
            app->default_dt = dt;
            app->run_state = cli_options.count("pause") ? PAUSE : RUN;
            if(cli_options.count("scene-max-frames")){
                app->max_animation_frames = cli_options["scene-max-frames"].as<int>();
                BOOST_LOG_TRIVIAL(debug) << "max-animation-frames: " << app->max_animation_frames;
            }
            if(cli_options.count("scene-max-seconds")){
                app->max_animation_seconds = cli_options["scene-max-seconds"].as<double>();
                BOOST_LOG_TRIVIAL(debug) << "max-animation-seconds: " << app->max_animation_seconds;
            }
            app->drawAll();
            app->setVisible(true);
            BOOST_LOG_TRIVIAL(info) << "Entering mainloop";
            nanogui::mainloop();
            BOOST_LOG_TRIVIAL(info) << "Simulation time: " << double(app->end_time - app->start_time)/CLOCKS_PER_SEC;
            BOOST_LOG_TRIVIAL(info) << "Simulated frames: " << app->total_frames;
            BOOST_LOG_TRIVIAL(info) << "Min fps: " << app->min_fps;
            BOOST_LOG_TRIVIAL(info) << "Max fps: " << app->max_fps;
            BOOST_LOG_TRIVIAL(info) << "Avg fps: " << app->total_frames / double(app->end_time - app->start_time)*CLOCKS_PER_SEC;
            if(physicsScene.fluid.get() != nullptr){
                BOOST_LOG_TRIVIAL(info) << "Fluid particles: " << physicsScene.fluid->particles_position().rows();
                BOOST_LOG_TRIVIAL(info) << "Boundary particles: " << physicsScene.fluid->boundary_position().rows();
            }
        }

        BOOST_LOG_TRIVIAL(info) << "Entering shutdown";
        nanogui::shutdown();
    } catch (const std::runtime_error &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
        #if defined(_WIN32)
            MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
        #else
            BOOST_LOG_TRIVIAL(fatal) << error_msg;
        #endif
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug) << "Terminating main";

    return 0;
}
