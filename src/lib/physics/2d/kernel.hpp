#pragma once


namespace GooBalls {
namespace d2 {
namespace Physics {

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
