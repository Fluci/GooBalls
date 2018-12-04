#pragma once

#include "types.hpp"
#include "scene.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

/// based on CFL condition: dt = lambda * h /max(sqrt(K), v_max)
Float recommendedTimeStep(const Scene& scene, Float lambda = 0.4);

}

}

} 
