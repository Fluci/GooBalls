#pragma once

#include "fluid.hpp"

namespace GooBalls {
namespace d2 {
namespace Render {

/**
 * Fluid who's particles will be rendered as disks.
 **/
class DiskFluid : public Fluid {
public:
    DiskFluid(std::shared_ptr<Fluid::Coordinates> ptr, std::shared_ptr<Fluid::Coordinates> boundary);
    /**
    * Renders the fluid's particles as disks according to their radius
    */
    virtual void render() const;
};

} // Render

} // d2

} // GooBalls

