#pragma once

#include "types.hpp"
#include "fluid.hpp"
#include "mesh.hpp"
#include "room.hpp"

#include <vector>
#include <Box2D/Box2D.h>

namespace GooBalls {
namespace d2 {
namespace Physics {

struct Scene {
    typedef Eigen::Matrix<FloatPrecision, 1, 2> Gravity;
    std::unique_ptr<Fluid> fluid;
    std::vector<Mesh> meshes;
	Gravity gravity = Gravity(0.0f, -9.81f);
	b2World world = b2World(b2Vec2(0.0,0.0));
    Room room;
};

} // Physics

} // d2

} // GooBalls

