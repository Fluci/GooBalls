/**
* Maintainer: Virginia Ramp
*/

#pragma once

#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

#include <Box2D/Box2D.h>
#include <vector>

#include "generic/types.hpp"
#include "physics/2d/scene.hpp"
#include "rendering/2d/scene.hpp"

namespace GooBalls {

using namespace d2;

class SceneLoader {
private:
    double scale;
    double transl_x;
    double transl_y;
    bool m_convertMeshToBoundingBox = false;
public:
/**
* Loads a scene from the scenes folder. For more info, look at the .cpp file's description.
*/
bool loadScene(Physics::Scene& physScene, Render::Scene& renderScene, std::string sceneName);

void setEmptyFluid(Physics::Scene& physScene, Render::Scene& renderScene) const;

void readFluid(Physics::Scene& physScene, Render::Scene& renderScene, const Json::Value& fluid) const;

void readObject(Physics::Scene& physScene, Render::Scene& renderScene, const Json::Value& obj) const;

Coordinates2d readGrid(const Json::Value& grid) const;

Coordinates2d createGrid(int px, int py, Float gridSpacing) const;

Coordinates2d readCoordinates(const Json::Value& coords) const;

std::vector<b2Vec2> computeConvexHull(const Coordinates2d& vertices) const;

/// Computes the axis aligned bounding box
std::vector<b2Vec2> computeAABB(const Coordinates2d& vertices) const;

/// Debug function: Replaces the vertices and faces such that only two triangles, that represent the bounding box, exist
void convertMeshToBoundingBox(Coordinates2d vertices, TriangleList& faces) const;

};

} // GooBalls
