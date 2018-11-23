#include "kernel_poly6.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {


void Poly6::setH(FloatPrecision h) {
    m_h = h;
    m_A = 315.0/(64.0*M_PI)/std::pow(h, 9); 
}

void Poly6::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    auto sqNorm = rs.rowwise().squaredNorm().eval();
    assert(sqNorm.maxCoeff() <= m_h*m_h*1.01);
    auto diffH = m_h*m_h - sqNorm.array();
    auto diffH2 = diffH * diffH;
    assert(diffH.rows() == rs.rows());
    assert(diffH2.rows() == rs.rows());
    if(wResult != nullptr){
        // 315/(64*M_PI)/std::pow(h, 9) * std::pow(h*h - r^2, 3)
        //wResult->resize(rs.rows(), 1);
        *wResult = m_A * diffH2*diffH;
    }
    if(gradientResult != nullptr){
        // d/dx  (h^2 - r^2)^3
        // = d/dx (h^2 - (x^2 + y^2))^3
        // = 3 (h^2 - r^2)^2 * (-2x)
        // = -6x (h^2 - r^2)^2
        auto comm = (-6.0 * m_A) * diffH2;
        gradientResult->resize(rs.rows(), rs.cols());
        gradientResult->col(0) = rs.col(0).array() * comm.array();
        gradientResult->col(1) = rs.col(1).array() * comm.array();
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
        *laplacianResult = (-6.0 * m_A) * diffH * (2.0*m_h*m_h - 6.0*sqNorm.array());
        //*laplacianResult = laplacianResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
}


} // Physics
} // d2
} // GooBalls
