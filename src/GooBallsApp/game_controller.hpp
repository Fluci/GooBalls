#pragma once

#include "physics/2d/scene.hpp"
#include <atomic>

namespace GooBalls {

namespace d2 {

class GameController {
    std::atomic_bool move_up = false;
    std::atomic_bool move_left = false;
    std::atomic_bool move_right = false;
public:
    bool keyboardEvent(int key, int scancode, int action, int modifiers);
    void apply(Physics::Scene& scene);
};

} // 2d

} // GooBalls
