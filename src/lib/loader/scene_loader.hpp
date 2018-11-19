/**
* Maintainer: Virginia Ramp
*/

#pragma once

#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

#include "physics/2d/scene.hpp"
#include "rendering/2d/scene.hpp"

namespace GooBalls {

namespace SceneLoader {

using namespace d2;

/**
* Loads a scene from the scenes folder. For more info, look at the .cpp file's description.
*/
void loadScene(Physics::Scene& physScene, Render::Scene& aRenderScene, std::string sceneName);



} // SceneLoader

} // GooBalls
