#pragma once

#include "fluid.hpp"

namespace GooBalls {
namespace d2 {

/**
 * Fluid who's particles will be rendered as disks.
 **/
class DiskFluid : public Fluid {
public:
    DiskFluid(std::shared_ptr<Fluid::Coordinates> ptr);
    virtual void render() const;
};

} // d2

} // GooBalls

