#include "recommended_timestep.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {


/// based on CFL condition: dt = lambda * h /max(sqrt(K), v_max)
Float recommendedTimeStep(const Scene& scene, Float lambda) {
    const auto& vs = scene.fluid->particles_velocity();
    Float K = scene.fluid->stiffnessConstant();
    Float maxV = std::sqrt(vs.rowwise().squaredNorm().maxCoeff());
    return lambda * scene.fluid->h() / std::max(std::sqrt(K), maxV);
}

} // Physics
} // d2
} // GooBalls
