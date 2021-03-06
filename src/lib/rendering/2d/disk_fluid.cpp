#include "disk_fluid.hpp"
#include <memory>
#include <iostream>

namespace GooBalls {

namespace d2 {

namespace Render { 

DiskFluid::DiskFluid(std::shared_ptr<Fluid::Coordinates> ptr, std::shared_ptr<Fluid::Coordinates> boundary) : Fluid(ptr, boundary) {
    // empty
}

void DiskFluid::render() const {
    auto& partCoords = particles_position();
    auto& partColors = particles_color();
    std::cout << "Got " << partCoords.rows() << " particles:\n";
    for(int i = 0; i < partCoords.rows(); ++i){
        std::cout << "xy: " << partCoords(i, 0) << ", " << partCoords(i, 1) << ", ";
        std::cout << "rgb: " << partColors(i, 0) << ", " << partColors(i, 1) << ", " << partColors(i, 2) << ", ";
        std::cout << "r: " << particles_radius()[i] << "\n";
    }
}

} // Render

} // d2

} // GooBalls
