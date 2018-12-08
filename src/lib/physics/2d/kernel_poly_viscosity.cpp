#include "kernel_poly_viscosity.hpp"


namespace GooBalls {
namespace d2 {
namespace Physics {


void PolyViscosity::setH(Float h) {
    m_h = h;
    m_A2d = 60 / (M_PI * std::pow(h, 5));
    m_A1d = 12.0/std::pow(h, 4);
    //m_A = 45.0 / (M_PI * std::pow(h, 6));
}


void PolyViscosity::compute1d(
        const Coordinates1d& r2,
        Coordinates1d* wResult,
        Coordinates1d* gradientResult,
        Coordinates1d* laplacianResult) const {
    Float h = m_h;
    Float A = m_A1d;
    assert(r2.maxCoeff() <= h*h*1.01);
    Coordinates1d r1 = r2.array().sqrt();

    Float h2 = h*h;
    if(wResult != nullptr){
        // -h^2x/2 + hx^2/2 - x^3/6 + h^3/6
        Float h3 = h*h2;
        Coordinates1d r3 = r2.array() * r1.array();
        *wResult = A*(-h2 * r1 / 2 + h * r2/2 - r3/6).array() + A*h3/6;
    }
    if(gradientResult != nullptr){
        // hx - x^2/2 - h^2/2
        gradientResult->derived().resize(r2.rows(), Eigen::NoChange);
        gradientResult->array().col(0) = A*(h*r1 - r2/2).array() - A*h2/2;
    }
    if(laplacianResult != nullptr){
        // h - x
        *laplacianResult = A * (h - r1.array());
    }
}

/// 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r + epsilon) - 1)
///
void PolyViscosity::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Coordinates1d r2 = rs.rowwise().squaredNorm();
    Float A = m_A2d;
	Float h = m_h;
    Float epsilon = 0.00001;

    Coordinates1d r1 = r2.array().sqrt();
    Coordinates1d diffH = h - r1.array();
    Coordinates1d diffH2 = diffH.array() * diffH.array();
    auto r1Inv = 1.0/(r1.array() + h*epsilon);
    if(wResult != nullptr){
        // -h^2x/2 + hx^2/2 - x^3/6 + h^3/6
        Float h2 = h*h;
        Float h3 = h*h2;
        Coordinates1d r3 = r2.array() * r1.array();
        *wResult = A*(-h2 * r1 / 2 + h * r2/2 - r3/6).array() + A*h3/6;
    }/*
    if(gradientResult != nullptr){
        Coordinates2d rsN = rs.array().colwise() / (r2.array().sqrt() + 0.000001);
        gradientResult->col(1) = rsN.col(1).array() * gradientResult->col(0).array();
        gradientResult->col(0) = rsN.col(0).array() * gradientResult->col(0).array();
    }*/
    if(gradientResult != nullptr){
        auto comm = (-3*A) * diffH2;
        *gradientResult = rs.array().colwise() * r1Inv;
        *gradientResult = gradientResult->array().colwise() * comm.array();
    }
    if(laplacianResult != nullptr){
        Float h2 = h*h;
        *laplacianResult = -3*A * (h2 * r1Inv + 3.0 * r1.array() - 4 * h);
        //*laplacianResult = A * (h - rs.rowwise().norm().array());
        //*laplacianResult = A * (m_h - r1.array());
    }
}


} // Physics
} // d2
} // GooBalls
