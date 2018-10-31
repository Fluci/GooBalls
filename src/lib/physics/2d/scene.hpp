#pragma once

#include "types.hpp"
#include "fluid.hpp"
#include "mesh.hpp"
#include <vector>

namespace GooBalls {
namespace d2 {
namespace Physics {

struct Scene {
    std::unique_ptr<Fluid> fluid;
    std::vector<Mesh> meshes;
};

} // Physics

} // d2

} // GooBalls

