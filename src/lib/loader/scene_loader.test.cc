#include "scene_loader.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

using namespace boost::filesystem;
using namespace GooBalls;

BOOST_AUTO_TEST_CASE(scene_loader_test_all_scenes){
    SceneLoader loader;
    path p("../examples/scenes/");
    for(auto& entry : boost::make_iterator_range(directory_iterator(p), {})){
        Physics::Scene phys;
        Render::Scene render;
        path pentry = entry;
        BOOST_LOG_TRIVIAL(info) << "Loading " << pentry;
        loader.loadScene(phys, render, pentry.string());
    }

}
