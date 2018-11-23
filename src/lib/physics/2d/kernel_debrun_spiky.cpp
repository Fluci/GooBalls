#include "kernel_debrun_spiky.hpp"

#include <iostream>

namespace GooBalls {
namespace d2 {
namespace Physics {

FloatPrecision epsilon = 0.000001;

void DebrunSpiky::setH(FloatPrecision h) {
    m_h = h;
    m_A = 40.0/M_PI/std::pow(h, 3);
}

void DebrunSpiky::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Coordinates1d sqNorm = rs.rowwise().squaredNorm();
    assert(sqNorm.maxCoeff() <= m_h*m_h*1.01);
    Coordinates1d norm = sqNorm.array().sqrt();
    Coordinates1d invNorm = 1.0 / (norm.array() + epsilon);
    Coordinates1d diffH = m_h - norm.array();
    Coordinates1d diffH2 = diffH.array() * diffH.array();
    assert(invNorm.rows() == rs.rows());
    assert(diffH.rows() == rs.rows());
    assert(diffH2.rows() == rs.rows());
    if(wResult != nullptr){
    // 15/(pi*h^6) (h-r)^3
        wResult->resize(rs.rows(), 1);
        *wResult = m_A * diffH2.array()*diffH.array();
    }
    if(gradientResult != nullptr){
        // d/dx * (h - sqrt(x^2+y^2))^3
        // = 3*(h - (x^2 + y^2)^1/2)^2  *  (-1/2*(x^2 + y^2)^(-1/2)) * 2 x
        // = -3*x*(h - |r|)^2/|r|
        auto comm = (-3*m_A) * diffH2.array() * invNorm.array();
        gradientResult->resize(rs.rows(), rs.cols());
        gradientResult->col(0) = comm.array() * rs.col(0).array();
        gradientResult->col(1) = comm.array() * rs.col(1).array();
        *gradientResult = gradientResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
    if(laplacianResult != nullptr){
        // d/dx * (x * h^2 / |r| - 2h*x + |r|*x)
        // = h^2*Ax - 2*h + Bx
        //
        // details:
        // Ax = d/dx * rx / |r|
        // = d/dx * rx * (rx*rx + ry*ry)^(-1/2)
        // = 1 / |r| - 1/2 rx * |r|^(-3/2) * 2*rx
        // = 1/|r| - rx^2 |r|^(-3/2)
        // = ry^2 / (rx^2 + ry^2)^(3/2)
        //
        // Bx = d/dx * |r| * rx
        // = d/dx * (rx*rx+ry*ry)^(1/2) * rx
        // = 1/2 (rx*rx + ry*ry)^(-1/2) * 2*rx * rx + |r|
        // = 1 / |r| * rx^2 + |r|
        // = rx^2 / |r| + |r|^2/|r|
        // = (rx^2 + rx^2 + ry^2)/|r|
        // = (2 rx^2 + ry^2) / |r|
        //
        // C = (rx^2 + ry^2)^(-3/2)
        // Ax = ry^2 / (rx^2 + ry^2)^(3/2) = ry^2 * C
        // Ay = rx^2 / (rx^2 + ry^2)^(3/2) = rx^2 * C
        // Bx = (2 rx^2 + ry^2) / |r|
        // By = (rx^2 + 2 ry^2) / |r|
        // laplacian
        // = h^2 * Ax - 2*h + Bx + h^2 Ay - 2*h + By
        // = h^2 * (Ax + Ay) - 4*h + Bx + By
        // = h^2 * C * (ry^2 + rx^2) - 4*h + 3*(rx^2 + ry^2)/|r|
        // = h^2 * (rx^2 + ry^2)^(-1/2) - 4*h + 3*r^2/|r|
        // = h^2 * 1/|r| - 4*h + 3 * |r|

        laplacianResult->resize(rs.rows(), 1);
        *laplacianResult = (m_h*m_h * invNorm) + (3.0*norm) ;
        *laplacianResult = laplacianResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
        *laplacianResult = (-3.0*m_A)*(laplacianResult->array() - 4.0*m_h);
    }
}


} // Physics
} // d2
} // GooBalls
