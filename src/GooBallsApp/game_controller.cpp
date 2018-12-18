#include "game_controller.hpp"
#include <nanogui/glutil.h>

namespace GooBalls {

namespace d2 {

GameController::GameController() : move_left(false), move_right(false), move_up(false) {
    // empty
}

bool GameController::keyboardEvent(int key, int scancode, int action, int modifiers){
    if (key == GLFW_KEY_A) {
        move_left = action != GLFW_RELEASE;
    }
    if (key == GLFW_KEY_D) {
        move_right = action != GLFW_RELEASE;
    }
    if (key == GLFW_KEY_W) {
        move_up = action != GLFW_RELEASE;
    }
    if (key == GLFW_KEY_S) {
        move_down = action != GLFW_RELEASE;
    }
    return true;
}

void GameController::apply(Physics::Scene& scene){
    if(move_left && move_right){
        move_left = false;
        move_right = false;
    }
    bool moveByForce = false;
    if(moveByForce){
        if (move_left) {
            scene.fluid->particles_external_force().col(0).array() = -1000;
            scene.fluid->particles_external_force().col(1).array() = 0;
        }
        if (move_right) {
            scene.fluid->particles_external_force().col(0).array() = 1000;
            scene.fluid->particles_external_force().col(1).array() = 0;
        }
        if (move_up) {
            scene.fluid->particles_external_force().col(0).array() = 0;
            scene.fluid->particles_external_force().col(1).array() = 1000;
        }
        if (move_up) {
            scene.fluid->particles_external_force().col(0).array() = 0;
            scene.fluid->particles_external_force().col(1).array() = -1000;
        }

        if(!move_left && !move_right && !move_up && !move_down){
            scene.fluid->particles_external_force().array() = 0;
        }
    } else {
        Float vMax = 2;
        Float acc = 0.01;

        auto& vs = scene.fluid->particles_velocity();
        if(move_left){
            vs.col(0) = vs.col(0).array()*(1-acc) - vMax * acc;
        }
        if(move_right){
            vs.col(0) = vs.col(0).array()*(1-acc) + vMax * acc;
        }
        if(move_up){
            vs.col(1) = vs.col(1).array()*(1-acc) + vMax * acc;
        }
    }
}

} // d2

} // GooBalls
