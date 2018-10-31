#pragma once

#include "types.hpp"
#include "scene.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {

class Engine {
public:
    /**
    * Modifies the objects in `scene` s.t. they move as if `dt` time passed
    **/
    void advance(Scene& scene, TimeStep dt);
};

} // Physics
} // d2
} // GooBalls
