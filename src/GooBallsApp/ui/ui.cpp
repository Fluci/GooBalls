#include "ui.h"

#include <boost/log/trivial.hpp>

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/entypo.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <iostream>
#include <string>

// Includes for the GLTexture class.
#include <cstdint>
#include <memory>
#include <utility>

#include "physics/2d/recommended_timestep.hpp"

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#define STB_IMAGE_IMPLEMENTATION
//#include <stb_image.h>

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif
#include "rendering/2d/disk_fluid.hpp"

namespace GooBalls {

namespace d2 {

using namespace nanogui;

ExampleApplication::ExampleApplication(Physics::Engine& physicsEngine, Render::Engine& renderEngine,
        Physics::Scene& physicsScene, Render::Scene& renderScene)
        : nanogui::Screen(Eigen::Vector2i(900, 900), "GooFBalls", false),
        m_physicsEngine(physicsEngine), m_physicsScene(physicsScene),
        m_renderEngine(renderEngine), m_renderScene(renderScene) {

    m_renderEngine.init();
    frames.push_back(clock());

    fps_label = new Label(this, "000 fps");
    fps_label->setColor({0,0,0,255});

    setBackground({190, 190, 255, 255});
    performLayout();
}

bool ExampleApplication::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers))
        return true;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        setVisible(false);
        return true;
    }


    return m_controller.keyboardEvent(key, scancode, action, modifiers);
}

void ExampleApplication::drawContents() {
    clock_t current = clock();
    clock_t previous = frames.back();
    clock_t lastSec = current - CLOCKS_PER_SEC;
    while(frames.front() < lastSec){
        frames.pop_front();
    }
    fps = frames.size();
    fps_label->setCaption(std::to_string(fps) + " fps");
    frames.push_back(current);
    //std::cout << "Fps: " << fps << "\n";
    double simulationSpeed = 0.5; // 1 = real time, 0.5: slomotion, 2: faster than real time
    /// How large of a total time step should be simulated?
    double target = std::min(double(current - previous)/CLOCKS_PER_SEC, 1/60.0)*simulationSpeed;
    /// dt: largest possible timestep, for which the simulation stays
    /// stable
    double dt = default_dt;
    if(use_recommended_timestep){
        dt = Physics::recommendedTimeStep(m_physicsScene);
    }
    BOOST_LOG_TRIVIAL(trace) << "Used timestep: " << dt;
    int n = std::max(1.0, std::ceil(target/dt));
    for(int i = 0; i < n; ++i){
        m_controller.apply(m_physicsScene);
        m_physicsEngine.advance(m_physicsScene, dt);
    }
    // for the moment only one fluid is supported
    if(!m_renderScene.fluids.empty()){
        assert(m_renderScene.fluids.size() == 1);
        auto& col = m_renderScene.fluids[0]->particles_color();
        const auto& ps = m_physicsScene.fluid->particles_pressure();
        auto& rad = m_renderScene.fluids[0]->particles_radius();
        const auto& rho = m_physicsScene.fluid->particles_density();
        const auto& rho0 = m_physicsScene.fluid->rest_density();
        assert(col.rows() == ps.rows());
        // pressure dependent color
        col.col(0).array() = 1.0;
        col.col(1) = 1.0/(1+0.01*(ps.array()).sqrt());
        col.col(2) = col.col(1);
        assert(rad.rows() == rho.rows());
        // density dependent size
        rad = rho0/(rho.array())*0.015;
        //rad = rho0/rho.array().pow(.5)*0.0003;
    }
    m_renderEngine.render(m_renderScene);
}

void ExampleApplication::draw(NVGcontext *ctx) {
    /* Draw the user interface */
    Screen::draw(ctx);
}

} // d2

} // GooBalls
