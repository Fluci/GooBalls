#include "render.hpp"

#pragma once

namespace GooBalls {
namespace d2 {

/**
 * Implementation of `Render` s.t. everything is rendered using OpenGL
 * */
class OpenGLRender : public Render {
public:
	virtual void render(const RenderScene& scene);
};

} // d2
} // GooBalls
