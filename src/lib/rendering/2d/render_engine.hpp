#pragma once

#include "render_scene.hpp"

namespace GooBalls {
namespace d2 {
/**
 * Know how to render a RenderScene object to screen
 * */
class RenderEngine {
public:
/**
 * Takes a RenderScene object and creates an image from it.
 * It also has the responsibility to display the image on screen.
 *
 * */
	void render(const RenderScene& scene);
};

} // d2
} // GooBalls
