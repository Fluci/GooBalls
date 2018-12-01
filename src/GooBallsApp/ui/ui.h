#pragma once

#include <nanogui/glcanvas.h>
#include <nanogui/screen.h>


#include "physics/2d/engine.hpp"
#include "rendering/2d/engine.hpp"

namespace GooBalls {

namespace d2 {

class MyGLCanvas : public nanogui::GLCanvas {
public:
    MyGLCanvas(Widget *parent, Render::Engine& renderEngine, Render::Scene& renderScene);

    ~MyGLCanvas() = default;

    virtual void drawGL() override;

private:
    Render::Engine& r_renderEngine;
    Render::Scene& r_renderScene;
};


class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication(Physics::Engine& physicsEngine, Render::Engine& renderEngine,
            Physics::Scene& physicsScene, Render::Scene& renderScene);

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);

    virtual void draw(NVGcontext *ctx);

private:
    MyGLCanvas *mCanvas;

    Physics::Engine& m_physicsEngine;
    Physics::Scene& m_physicsScene;

    Render::Engine& m_renderEngine;
    Render::Scene& m_renderScene;
};

} // d2

} // GooBalls
