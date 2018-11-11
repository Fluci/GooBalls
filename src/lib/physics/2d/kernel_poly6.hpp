#pragma once

#include "types.hpp"
#include "kernel.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

/**
 * Implements the poly 6 kernel:
 *
 * 315/(64*M_PI)/(h^9) * (h^2 - r)^3
 * */
class Poly6: public Kernel {
private:
    FloatPrecision m_h;
    FloatPrecision m_A;
public:
    virtual void setH(FloatPrecision h);
    virtual void compute(
            const Coordinates2d& rs,
            Coordinates1d* W,
            Coordinates2d* gradient,
            Coordinates1d* laplacian) const;
};

} // Physics

} // d2

} // GooBalls

