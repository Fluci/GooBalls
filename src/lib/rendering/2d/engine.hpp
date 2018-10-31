#pragma once

#include "scene.hpp"

namespace GooBalls {
namespace d2 {
namespace Render {
/**
 * Know how to render a RenderScene object to screen
 * */
class Engine {
public:
/**
 * Takes a RenderScene object and creates an image from it.
 * It also has the responsibility to display the image on screen.
 *
 * */
	void render(const Scene& scene);
};
} // Render
} // d2
} // GooBalls
