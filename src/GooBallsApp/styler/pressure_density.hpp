#pragma once
#include "styler.hpp"

namespace GooBalls {

using namespace d2;

class PressureDensity: public Styler {
public:
    virtual void shapeScene(const Physics::Scene& physScene, Render::Scene& renderScene) const override;
};

}
