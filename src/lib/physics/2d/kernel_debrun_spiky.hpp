#pragma once

#include "types.hpp"
#include "kernel.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

class DebrunSpiky: public Kernel {
private:
    Float m_h;
    Float m_A1d;
    Float m_A2d;
public:
    virtual void setH(Float h);
    virtual void compute(
            const Coordinates2d& rs,
            Coordinates1d* W,
            Coordinates2d* gradient,
            Coordinates1d* laplacian) const override;
    virtual void compute1d(
            const Coordinates1d& squaredNorm,
            Coordinates1d* wResult,
            Coordinates1d* gradientResult,
            Coordinates1d* laplacianResult) const override;
};

} // Physics

} // d2

} // GooBalls

