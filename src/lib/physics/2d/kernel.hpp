#pragma once

#include "types.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {

/**
 * Properties a kernel should have:
 * C^0, C^1 continuous
 * evaluated at r = h, the value should be zero
 * Kernel must be normalized: int_X W(|x - x_i|) dx = 1
 */
class Kernel {
public:
    /**
     * Computes the kernel values for the difference vector r_i - r_j
     *
     * Calculates the values for all out-arguments != nullptr
     * */
    virtual void compute(
            const Coordinates2d& rs, 
            Coordinates1d* wResult, 
            Coordinates2d* gradientResult, 
            Coordinates1d* laplacianResult) const = 0;
    virtual Float scale2d() const = 0;
    virtual Float computeValue(const TranslationVector rs) const = 0;
    virtual TranslationVector computeGradient(const TranslationVector rs) const = 0;
    virtual Float computeLaplacian(const TranslationVector rs) const = 0;
    virtual void compute1d(
            const Coordinates1d& squaredNorm,
            Coordinates1d* wResult,
            Coordinates1d* gradientResult,
            Coordinates1d* laplacianResult) const = 0;
    /**
     * Gives the kernel the chance to precompute things.
     * */
    virtual void setH(Float h) = 0;
};


} // Physics

} // d2

} // Kernel
