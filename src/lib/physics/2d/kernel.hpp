#pragma once


namespace GooBalls {
namespace d2 {
namespace Physics {

/**
 * Properties a kernel should have:
 * C^0, C^1 continuous
 * evaluated at r = h, the value should be zero
 * Kernel must be normalized: int_X W(|x - x_i|) dx = 1
 * TODO: create Test suite for this
 */
class Kernel {
public:
    /** 
     * Calculates the values for all out-arguments != nullptr
     * */
    virtual void compute(
            const Coordinates2d& rs, 
            Coordinates1d* wResult, 
            Coordinates2d* gradientResult, 
            Coordinates1d* laplacianResult) const = 0;
    /**
     * Gives the kernel the chance to precompute things.
     * */
    virtual void setH(FloatPrecision h) = 0;
};


} // Physics

} // d2

} // Kernel
