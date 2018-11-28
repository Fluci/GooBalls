/*
    src/example4.cpp -- C++ version of an example application that shows
    how to use the OpenGL widget. For a Python implementation, see
    '../python/example4.py'.

    NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
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
#include <nanogui/glcanvas.h>
#include <iostream>
#include <string>

// Includes for the GLTexture class.
#include <cstdint>
#include <memory>
#include <utility>

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

#include "physics/2d/engine.hpp"
#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"

namespace GooBalls {

namespace d2 {

using std::endl;

class MyGLCanvas : public nanogui::GLCanvas {
public:
    MyGLCanvas(Widget *parent, Render::Engine& renderEngine, Render::Scene& renderScene) 
        : nanogui::GLCanvas(parent), r_renderEngine(renderEngine), r_renderScene(renderScene) { }
    ~MyGLCanvas() { }

    virtual void drawGL() override {
        r_renderEngine.render(r_renderScene);
    }

private:
    Render::Engine& r_renderEngine;
    Render::Scene& r_renderScene;
};


class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication(Physics::Engine& physicsEngine, Render::Engine& renderEngine,
            Physics::Scene& physicsScene, Render::Scene& renderScene) 
            : nanogui::Screen(Eigen::Vector2i(900, 900), "NanoGUI Test", false),
            m_physicsEngine(physicsEngine), m_renderEngine(renderEngine),
            m_physicsScene(physicsScene), m_renderScene(renderScene) {
        using namespace nanogui;

        Window *window = new Window(this, "GooFBalls");
        window->setPosition(Vector2i(15, 15));
        window->setLayout(new GroupLayout());

        mCanvas = new MyGLCanvas(window, m_renderEngine, m_renderScene);
        mCanvas->setBackgroundColor({190, 190, 255, 255});
        mCanvas->setSize({800, 800});

        Widget *tools = new Widget(window);
        tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 5));

        Button *b0 = new Button(tools, "Random Color");
        b0->setCallback([this]() { mCanvas->setBackgroundColor(Vector4i(rand() % 256, rand() % 256, rand() % 256, 255)); });

        performLayout();

        m_renderEngine.init();
    }

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) {
        if (Screen::keyboardEvent(key, scancode, action, modifiers))
            return true;
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            setVisible(false);
            return true;
        }
        if (key == GLFW_KEY_A && action != GLFW_RELEASE) {
            m_physicsScene.fluid->particles_external_force().col(0).array() = -10000;
            m_physicsScene.fluid->particles_external_force().col(1).array() = 0;
        } else if (key == GLFW_KEY_D && action != GLFW_RELEASE) {
            m_physicsScene.fluid->particles_external_force().col(0).array() = 10000;
            m_physicsScene.fluid->particles_external_force().col(1).array() = 0;
        } else if (key == GLFW_KEY_W && action != GLFW_RELEASE) {
            m_physicsScene.fluid->particles_external_force().col(0).array() = 0;
            m_physicsScene.fluid->particles_external_force().col(1).array() = 10000;
        } else {
            m_physicsScene.fluid->particles_external_force().array() = 0;
        }
        return false;
    }

    virtual void draw(NVGcontext *ctx) {
        /// seconds per frame: TODO get from measurments or so
        double target = 1/200.0;
        /// dt: largest possible timestep, for which the simulation stays
        /// stable
        double dt = 0.001;
        int n = std::max(1.0, target/dt);
        for(int i = 0; i < n; ++i){
            m_physicsEngine.advance(m_physicsScene, dt);
        }
        if(!m_renderScene.fluids.empty()){
            auto& col = m_renderScene.fluids[0]->particles_color();
            const auto& ps = m_physicsScene.fluid->particles_pressure();
            auto& rad = m_renderScene.fluids[0]->particles_radius();
            const auto& rho = m_physicsScene.fluid->particles_density();
            const auto& rho0 = m_physicsScene.fluid->rest_density();
            assert(col.rows() == ps.rows());
            col.col(0).array() = 1.0;
            col.col(1) = 1.0/(1+0.01*(ps.array()).sqrt());
            col.col(2) = col.col(1);
            assert(rad.rows() == rho.rows());
            rad = rho0/(rho.array())*0.015;
            //rad = rho0/rho.array().pow(.5)*0.0003;
        }
        /* Draw the user interface */
        Screen::draw(ctx);
    }
private:
    MyGLCanvas *mCanvas;

    Physics::Engine& m_physicsEngine;
    Physics::Scene& m_physicsScene;

    Render::Engine& m_renderEngine;
    Render::Scene& m_renderScene;
};

} // d2

} // GooBalls
