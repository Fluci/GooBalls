#pragma once

#include "physics/2d/scene.hpp"
#include <atomic>

namespace GooBalls {

namespace d2 {

class GameController {
    std::atomic_bool move_up;
    std::atomic_bool move_down;
    std::atomic_bool move_left;
    std::atomic_bool move_right;
public:
    GameController();
    bool keyboardEvent(int key, int scancode, int action, int modifiers);
    void apply(Physics::Scene& scene);
};

} // 2d

} // GooBalls
