#pragma once

#include <nanogui/glcanvas.h>
#include <nanogui/screen.h>


#include "physics/2d/engine.hpp"
#include "rendering/2d/engine.hpp"
#include "game_controller.hpp"

namespace GooBalls {

namespace d2 {


class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication(Physics::Engine& physicsEngine, Render::Engine& renderEngine,
            Physics::Scene& physicsScene, Render::Scene& renderScene);

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);

    virtual void draw(NVGcontext *ctx);
    virtual void drawContents();


private:
    GameController m_controller;

    Physics::Engine& m_physicsEngine;
    Physics::Scene& m_physicsScene;

    Render::Engine& m_renderEngine;
    Render::Scene& m_renderScene;
};

} // d2

} // GooBalls
