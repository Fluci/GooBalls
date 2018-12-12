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
#include "styler/no_style.hpp"

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
        styler(std::make_unique<NoStyle>()),
        m_physicsEngine(physicsEngine), m_physicsScene(physicsScene),
        m_renderEngine(renderEngine), m_renderScene(renderScene) {

    m_renderEngine.init();
    frames.push_back(clock());

    fps_label = new Label(this, "000 fps");
    fps_label->setColor({0,0,0,255});

    setBackground({190, 190, 255, 255});
    performLayout();
    start_time = clock();
}

bool ExampleApplication::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers))
        return true;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        setVisible(false);
        return true;
    }
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS){
        run_state = run_state != RUN ? RUN : PAUSE;
        BOOST_LOG_TRIVIAL(info) << "run state: " << run_state;
        return true;
    }
    if (key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
        run_state = modifiers & GLFW_MOD_SHIFT ? ONE_TIME_STEP : ONE_FRAME;
        BOOST_LOG_TRIVIAL(info) << "run state: " << run_state;
        return true;
    }
    if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
        slow_motion = 1.5*slow_motion;
        if(std::abs(1.0 - slow_motion) < 0.05){slow_motion = 1.0;}
        BOOST_LOG_TRIVIAL(info) << "slow_motion: " << slow_motion;
        return true;
    }
    if(key == GLFW_KEY_DOWN && action == GLFW_PRESS){
        slow_motion = 1.0/1.5*slow_motion;
        if(std::abs(1.0 - slow_motion) < 0.05){slow_motion = 1.0;}
        BOOST_LOG_TRIVIAL(info) << "slow_motion: " << slow_motion;
        return true;
    }


    return m_controller.keyboardEvent(key, scancode, action, modifiers);
}

void ExampleApplication::drawContents() {
    clock_t current = clock();
    end_time = current;
    clock_t previous = frames.back();
    clock_t lastSec = current - CLOCKS_PER_SEC;
    while(frames.front() < lastSec){
        frames.pop_front();
    }
    total_frames++;
    frames.push_back(current);
    fps = std::floor(frames.size()/double(frames.back() - frames.front())*CLOCKS_PER_SEC);
    if(fps < 120){
        min_fps = std::min(min_fps, fps);
        max_fps = std::max(max_fps, fps);
    }
    assert(frames.size() > 0);
    fps_label->setCaption(std::to_string(fps) + " fps");
    if(run_state != PAUSE){
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
        int n = std::max(1.0, std::ceil(slow_motion*target/dt));
        if(run_state == ONE_TIME_STEP){
            n = 1;
        }
        if(n == 1){
            BOOST_LOG_TRIVIAL(info) << "Performing one timestep of " << dt;
        }
        for(int i = 0; i < n; ++i){
            m_controller.apply(m_physicsScene);
            m_physicsEngine.advance(m_physicsScene, dt);
        }
        if(run_state == ONE_FRAME || run_state == ONE_TIME_STEP){
            run_state = PAUSE;
        }
    }
    styler->shapeScene(m_physicsScene, m_renderScene);
    m_renderEngine.render(m_renderScene);
}

void ExampleApplication::draw(NVGcontext *ctx) {
    /* Draw the user interface */
    Screen::draw(ctx);
}

} // d2

} // GooBalls
