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

#include "rendering/2d/engine.hpp"
#include "rendering/2d/disk_fluid.hpp"

using std::endl;

class MyGLCanvas : public nanogui::GLCanvas {
public:
    MyGLCanvas(Widget *parent) : nanogui::GLCanvas(parent) { }
    ~MyGLCanvas() { }

    virtual void drawGL() override {
        std::shared_ptr<GooBalls::Coordinates2d> centers = std::make_shared<GooBalls::Coordinates2d>();
        centers->resize(4, 2);
        centers->row(0) << -0.45f,  0.45f;
        centers->row(1) <<  0.45f,  0.45f;
        centers->row(2) <<  0.45f, -0.45f;
        centers->row(3) << -0.45f, -0.45f;

        Eigen::VectorXf radii(4);
        radii << 0.1f, 0.2f, 0.3f, 0.4f;

        GooBalls::ColorsFloatRGB colors(4, 3);
        colors.row(0) << 0.8f, 0.8f, 0.8f;
        colors.row(1) << 0.5f, 0.5f, 0.7f;
        colors.row(2) << 0.2f, 0.5f, 0.2f;
        colors.row(3) << 0.6f, 0.3f, 0.1f;

        auto fluid = std::make_unique<GooBalls::d2::Render::DiskFluid>(centers);
        fluid->particles_radius() = radii;
        fluid->particles_color() = colors;

        GooBalls::d2::Render::Scene renderScene;
        renderScene.fluids.push_back(std::move(fluid));
        m_renderEngine.render(renderScene);
    }

private:
    GooBalls::d2::Render::Engine m_renderEngine;
};


class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication() : nanogui::Screen(Eigen::Vector2i(600, 650), "NanoGUI Test", false) {
        using namespace nanogui;

        Window *window = new Window(this, "GooFBalls");
        window->setPosition(Vector2i(15, 15));
        window->setLayout(new GroupLayout());

        mCanvas = new MyGLCanvas(window);
        mCanvas->setBackgroundColor({80, 80, 100, 255});
        mCanvas->setSize({500, 500});

        Widget *tools = new Widget(window);
        tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 5));

        Button *b0 = new Button(tools, "Random Color");
        b0->setCallback([this]() { mCanvas->setBackgroundColor(Vector4i(rand() % 256, rand() % 256, rand() % 256, 255)); });

        performLayout();
    }

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) {
        if (Screen::keyboardEvent(key, scancode, action, modifiers))
            return true;
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            setVisible(false);
            return true;
        }
        return false;
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }
private:
    MyGLCanvas *mCanvas;
};
