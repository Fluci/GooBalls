#pragma once
#include "physics/2d/scene.hpp"
#include "rendering/2d/scene.hpp"

namespace GooBalls {

using namespace d2;

/**
 * @brief This class is responsible to change the appearance of scene elements according to the physics.
 */
class Styler {
public:
    virtual void shapeScene(const Physics::Scene& physScene, Render::Scene& renderScene) const = 0;
};

}

