#include "kernel_poly6.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {


void Poly6::setH(Float h) {
    m_h = h;
    m_A = 16.0/M_PI/std::pow(h, 6);
}

template <typename DerivedIn, typename DerivedOut1, typename DerivedOut2, typename DerivedOut3>
void computePoly6(
        const Eigen::MatrixBase<DerivedIn>& sqNorm,
        Eigen::MatrixBase<DerivedOut1>* wResult,
        Eigen::MatrixBase<DerivedOut2>* gradientResult,
        Eigen::MatrixBase<DerivedOut3>* laplacianResult,
        Float h,
        Float A) {
    assert(sqNorm.maxCoeff() <= h*h*1.01);
    Coordinates1d sqDiffH = h*h - sqNorm.array();
    Coordinates1d sqDiffH2 = sqDiffH.array() * sqDiffH.array();
    assert(sqDiffH.rows() == sqNorm.rows());
    assert(sqDiffH2.rows() == sqNorm.rows());
    if(wResult != nullptr){
        // 315/(64*M_PI)/std::pow(h, 9) * std::pow(h*h - r^2, 3)
        //wResult->resize(rs.rows(), 1);
        *wResult = A * sqDiffH2.array() * sqDiffH.array();
    }
    if(gradientResult != nullptr){
        // d/dx  (h^2 - r^2)^3
        // = d/dx (h^2 - (x^2 + y^2))^3
        // = 3 (h^2 - r^2)^2 * (-2x)
        // = -6x (h^2 - r^2)^2
        gradientResult->derived().resize(sqNorm.rows(), Eigen::NoChange);
        gradientResult->col(0) = (-6.0 * A) * sqDiffH2;
        //*gradientResult = gradientResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
    if(laplacianResult != nullptr){
        // d/dx x (h^2 - r^2)^2
        // = (h^2 - r^2)^2 - x 2 (h^2 - r^2) * 2x
        // = (h^2 - r^2)^2 - 4x^2 (h^2 - r^2)
        // = (h^2 - r^2) (h^2 - r^2 - 4x^2)
        // = (h^2 - r^2) (h^2 - x^2 - y^2 - 4x^2)
        // = (h^2 - r^2) (h^2 - 5x^2 - y^2)
        // laplacian:
        // = (h^2 - r^2) (h^2 - r^2 - 4x^2) + (h^2 - r^2) (h^2 - r^2 - 4 y^2)
        // = (h^2 - r^2) (2h^2 - 2r^2 - 4(x^2 + y^2))
        // = (h^2 - r^2) 2(h^2 - r^2 - 2r^2)
        // = (h^2 - r^2) 2(h^2 - 3r^2)
        // = (h^2 - r^2) 2(h^2 - 3(x^2 + y^2))
        *laplacianResult = (-6.0 * A) * sqDiffH.array() * (2.0*h*h - 6.0*sqNorm.array());
        //*laplacianResult = laplacianResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
}

void Poly6::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Coordinates1d sqNorm = rs.rowwise().squaredNorm();
    computePoly6(rs.rowwise().squaredNorm(), wResult, gradientResult, laplacianResult, m_h, m_A);
    if(gradientResult != nullptr){
        gradientResult->col(1) = rs.col(1).array() * gradientResult->col(0).array();
        gradientResult->col(0) = rs.col(0).array() * gradientResult->col(0).array();
    }
}


} // Physics
} // d2
} // GooBalls
