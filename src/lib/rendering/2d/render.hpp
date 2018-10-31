#pragma once

#include "render_scene.hpp"

namespace GooBalls {
namespace d2 {
/**
 * Interface for a render engine
 *
 * */
class Render {
public:
/**
 * Takes a RenderScene object and creates an image from it.
 * It also has the responsibility to display the image on screen.
 *
 * */
	virtual void render(const RenderScene& scene) = 0;
};

} // d2
} // GooBalls
