#pragma once
#include "types.hpp"
#include "fluid.hpp"
#include "mesh.hpp"
#include <memory>
#include <vector>

namespace GooBalls {
namespace d2 {

struct Background {
	ColorFloatPrecision r;
	ColorFloatPrecision g;
	ColorFloatPrecision b;
};

/**
 * Coordinate sytem:
 * +x points to the right
 * +y points upwards
 * (0,0) is located at the lower left corner
 */
struct Camera {
	CoordinatePrecision x;
	CoordinatePrecision y;
	CoordinatePrecision width;
	CoordinatePrecision height;
};

/**
 * This class describes a 2D scene that should be rendered.
 *
 * */
struct RenderScene {
	std::vector<Fluid> fluids;
	std::vector<Mesh> meshes;
	Background background;
	Camera camera;
};
} // d2
} // GooBalls
