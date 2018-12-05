#pragma once

#include <nanogui/glcanvas.h>
#include <nanogui/screen.h>
#include <ctime>
#include <deque>


#include "physics/2d/engine.hpp"
#include "rendering/2d/engine.hpp"
#include "game_controller.hpp"

namespace GooBalls {

namespace d2 {

enum RUN_STATE {
    RUN,
    PAUSE,
    ONE_FRAME,
    ONE_TIME_STEP
};

class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication(Physics::Engine& physicsEngine, Render::Engine& renderEngine,
            Physics::Scene& physicsScene, Render::Scene& renderScene);

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);

    virtual void draw(NVGcontext *ctx);
    virtual void drawContents();



    bool use_recommended_timestep = false;
    double default_dt = 0.001;
    // the higher, the faster the simulation runs
    double slow_motion = 1.0;
    RUN_STATE run_state = RUN;
private:
    GameController m_controller;

    Physics::Engine& m_physicsEngine;
    Physics::Scene& m_physicsScene;

    Render::Engine& m_renderEngine;
    Render::Scene& m_renderScene;


    /// remembers the last frame render times
    /// needed to figure out the amout of phyiscs we should compute
    /// gives us exact fps
    std::deque<clock_t> frames;
    int fps = 0;
    nanogui::Label* fps_label;
};

} // d2

} // GooBalls
