#pragma once

#include "types.hpp"
#include "kernel.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

class DebrunSpiky: public Kernel {
private:
    Float m_h;
    Float m_A;
public:
    virtual void setH(Float h);
    virtual void compute(
            const Coordinates2d& rs,
            Coordinates1d* W,
            Coordinates2d* gradient,
            Coordinates1d* laplacian) const;
};

} // Physics

} // d2

} // GooBalls

