#pragma once

#include "abstract_sph.hpp"
#include "kernel.hpp"
#include <memory>

namespace GooBalls {

namespace d2 {

namespace Physics {

/**
 * Standard smooth particle hydrodynamics according to the Paper by Müller et al. from 2003
 * SSPH models a compressible fluid!
 * */
class IISPH : public AbstractSph {
private:
    std::unique_ptr<Kernel> m_kernelDensity;
    std::unique_ptr<Kernel> m_kernelPressure;
    std::unique_ptr<Kernel> m_kernelViscosity;
    Coordinates1d rhoAdv;
    Coordinates1d p0;
    Coordinates1d aii;
    Coordinates2d dii;

    void predictAdvection(Scene& scene, TimeStep dt, const Kernel& kernel);
    void pressureSolve(Scene& scene, TimeStep dt, const Kernel& kernel);
public:
    IISPH();
    virtual ~IISPH() = default;
    virtual void advance(Scene& scene, TimeStep dt);
    virtual void computeTotalForce(Scene& scene, TimeStep dt);
    void densityKernel(std::unique_ptr<Kernel>&& k);
    void pressureKernel(std::unique_ptr<Kernel>&& k);
    void viscosityKernel(std::unique_ptr<Kernel>&& k);

};

} // Physics

} // d2

} // GooBalls

