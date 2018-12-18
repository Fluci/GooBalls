#include "pressure_density.hpp"

#include <cassert>

namespace GooBalls {
using namespace d2;

void PressureDensity::shapeScene(const Physics::Scene& physicsScene, Render::Scene& renderScene) const {
    // for the moment only one fluid is supported
    if(renderScene.fluids.empty()){
        return;
    }
    assert(renderScene.fluids.size() == 1);
    auto& col = renderScene.fluids[0]->particles_color();
    const auto& ps = physicsScene.fluid->particles_pressure();
    auto& rad = renderScene.fluids[0]->particles_radius();
    const auto& rho = physicsScene.fluid->particles_density();
    const auto& rho0 = physicsScene.fluid->rest_density();
    assert(col.rows() == ps.rows());
    // pressure dependent color
    col.col(0).array() = 1.0;
    col.col(1) = 1.0/(1.0+0.001*(ps.array()).sqrt());
    col.col(2) = col.col(1);
    assert(rad.rows() == rho.rows());
    // density dependent size
    rad = (rho0/(rho.array())*0.015).max(0.001);
    //rad = rho0/rho.array().pow(.5)*0.0003;
}

}
