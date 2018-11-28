#pragma once

#include "abstract_sph.hpp"
#include "kernel.hpp"
#include "spatial/2d/neighborhood.hpp"
#include <memory>

namespace GooBalls {

namespace d2 {

namespace Physics {

using namespace Spatial;

/**
 * Standard smooth particle hydrodynamics according to the Paper by Müller et al. from 2003
 * SSPH models a compressible fluid!
 * */
class SSPH : public AbstractSph {
private:
    std::unique_ptr<Kernel> m_kernelDensity;
    std::unique_ptr<Kernel> m_kernelPressure;
    std::unique_ptr<Kernel> m_kernelViscosity;
    bool m_consider_boundary = true;
public:
    SSPH();
    virtual ~SSPH() = default;
    virtual void advance(Scene& scene, TimeStep dt);
    virtual void computeTotalForce(Scene& scene, TimeStep dt);
    virtual bool considerBoundary(bool consider);
    virtual bool considerBoundary() const;
    void densityKernel(std::unique_ptr<Kernel>&& k);
    void pressureKernel(std::unique_ptr<Kernel>&& k);
    void viscosityKernel(std::unique_ptr<Kernel>&& k);

};

} // Physics

} // d2

} // GooBalls

