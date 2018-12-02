#pragma once

#include "physics/2d/scene.hpp"

namespace GooBalls {

namespace d2 {

class GameController {
    bool move_up = false;
    bool move_left = false;
    bool move_right = false;
public:
    bool keyboardEvent(int key, int scancode, int action, int modifiers);
    void apply(Physics::Scene& scene);
};

} // 2d

} // GooBalls
