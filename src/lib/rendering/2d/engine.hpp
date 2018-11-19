#pragma once

#include "nanogui/opengl.h"
#include "nanogui/glutil.h"

#include "scene.hpp"

namespace GooBalls {
namespace d2 {
namespace Render {
/**
 * Know how to render a RenderScene object to screen
 * */
class Engine {
public:
    Engine();
    ~Engine();

    /**
     * Takes a RenderScene object and creates an image from it.
     * It also has the responsibility to display the image on screen.
     *
     * */
	void render(const Scene& scene);

    void init();
private:
    nanogui::GLShader m_particleShader;
};
} // Render
} // d2
} // GooBalls
