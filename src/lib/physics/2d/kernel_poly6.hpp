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
    Float m_h;
    Float m_A1d;
    Float m_A2d;
    Float m_epsilon = 0.000001;
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

