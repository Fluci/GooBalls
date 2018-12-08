#include "kernel_divergent_viscosity.hpp"


namespace GooBalls {
namespace d2 {
namespace Physics {


void DivergentViscosity::setH(Float h) {
    m_h = h;
    // TODO
    //m_A1d = 24.0/(m_h * (-19.0 + 12.0 * std::log(1.0 + 1.0/m_epsilon)));
    m_A1d = 1;
    m_A2d = 45.0 / (M_PI * std::pow(h, 6));
}


template <typename DerivedIn, typename DerivedOut1, typename DerivedOut2, typename DerivedOut3>
void computeDivergentViscosity(
        const Eigen::MatrixBase<DerivedIn>& r2,
        Eigen::MatrixBase<DerivedOut1>* wResult,
        Eigen::MatrixBase<DerivedOut2>* gradientResult,
        Eigen::MatrixBase<DerivedOut3>* laplacianResult,
        Float h,
        Float A,
        Float epsilon) {
    Float h2 = h*h;
    Float h3 = h*h2;
    assert(r2.maxCoeff() <= h*h*1.01);
    Coordinates1d r1 = r2.array().sqrt();
    Coordinates1d diffH = h - r1.array();
    Coordinates1d diffH2 = diffH.array() * diffH.array();
    auto r3 = r1.array() * r2.array();
    auto rpInv = 1.0/(r1.array()+h*epsilon);
    if(wResult != nullptr){
    	// A*(-r^3 / (2h^3) + r^2 / h^2 + h/2/(r + h*p) - 1)
        *wResult = A*(1.0/h2 * r2.array() -1.0/(2*h3) * r3.array() + h/2.0 * rpInv - 1);
    }
    if(gradientResult != nullptr){
    	// (2 r)/h^2 - (3 r^2)/(2 h^3) - h/(2 (h*p + r)^2)
        gradientResult->derived().resize(r2.rows(), Eigen::NoChange);
        auto rp2Inv = rpInv.array() * rpInv.array();
        gradientResult->array().col(0) = A*(2.0/h2 * r1.array() - 3.0/(2*h3) * r2.array() - h/2*rp2Inv);
    }
    if(laplacianResult != nullptr){
		// 2/h^2 - (3 r)/h^3 + h/(h*p + r)^3
        auto rp3Inv = rpInv.array() * rpInv.array() * rpInv.array();
        *laplacianResult = A * (2.0/h2 - 3/h3 * r1.array() + h*rp3Inv);
    }
}

void DivergentViscosity::compute1d(
        const Coordinates1d& squaredNorm,
        Coordinates1d* wResult,
        Coordinates1d* gradientResult,
        Coordinates1d* laplacianResult) const {
//    m_A = 45.0 / (M_PI * std::pow(h, 6));
    computeDivergentViscosity(squaredNorm, wResult, gradientResult, laplacianResult, m_h, m_A1d, m_epsilon);
}

/// 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r + epsilon) - 1)
///
void DivergentViscosity::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Coordinates1d sqNorm = rs.rowwise().squaredNorm();
    computeDivergentViscosity(sqNorm, wResult, gradientResult, laplacianResult, m_h, 1.0, m_epsilon);
    if(gradientResult != nullptr){
        Coordinates2d rsN = rs.array().colwise() / (sqNorm.array().sqrt() + 0.000001);
        gradientResult->col(1) = rsN.col(1).array() * gradientResult->col(0).array();
        gradientResult->col(0) = rsN.col(0).array() * gradientResult->col(0).array();
    }
}


} // Physics
} // d2
} // GooBalls
