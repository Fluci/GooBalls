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
    /// Box2D's velocity iteration count
    int velocity_iterations = 6;
    /// Box2D's position iteration count
    int position_iterations = 2;
};

} // Physics
} // d2
} // GooBalls
