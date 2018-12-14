#pragma once

#include "types.hpp"
#include "kernel.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

/**
 * Implements the viscosity kernel:
 *
 * 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r) - 1)
 * */
class PolyViscosity: public Kernel {
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
    virtual Float computeValue(const TranslationVector rs) const override;
    virtual TranslationVector computeGradient(const TranslationVector rs) const override;
    virtual Float computeLaplacian(const TranslationVector rs) const override;
    virtual void compute1d(
            const Coordinates1d& squaredNorm,
            Coordinates1d* wResult,
            Coordinates1d* gradientResult,
            Coordinates1d* laplacianResult) const override;
};

} // Physics

} // d2

} // GooBalls

