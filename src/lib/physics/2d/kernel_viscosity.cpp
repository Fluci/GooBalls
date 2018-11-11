#include "kernel_viscosity.hpp"


namespace GooBalls {
namespace d2 {
namespace Physics {


void Viscosity::setH(FloatPrecision h) {
    m_h = h;
    m_A = 15.0/(2.0*M_PI)/std::pow(h, 3); 
}

/// 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r) - 1)
///
/// WARNING: I don't get the same results for the laplacian as given in the paper.
/// We need to clarify this before using this kernel.
void Viscosity::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    auto sqNorm = rs.rowwise().squaredNorm().eval();
    auto norm = sqNorm.array().sqrt().eval();
    auto diffH = m_h*m_h - sqNorm.array();
    auto diffH2 = diffH * diffH;
    assert(diffH.rows() == rs.rows());
    assert(diffH2.rows() == rs.rows());
    FloatPrecision h2 = m_h*m_h;
    FloatPrecision h3 = m_h*h2;
    if(wResult != nullptr){
        // 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r) - 1)
        // ((-(x^2 + y^2)^(3/2) / (2h^3) + (x^2 + y^2) / h^2 + h/2 * (x^2 + y^2)^(-1/2)) - 1))
        wResult->resize(rs.rows(), 1);
        auto r3 = norm.array().pow(3.0);
        *wResult = m_A * (-r3 / (2*h3) + sqNorm.array() / h2 + (m_h / 2.0) / norm.array() - 1);
    }
    if(gradientResult != nullptr){
        // d/dx (-r^3 / (2h^3)          + r^2 / h^2         + h/2 * r^-1               - 1)
        // d/dx (-(x^2 + y^2)^1.5/(2h^3)+ (x^2 + y^2) / h^2 + h/2 * (x^2 + y^2)^(-1/2) - 1)
        // 
        // = x (-h^4 * (x^2 + y^2)^-1.5 + 4h - 3(x^2 + y^2)^0.5)/(2h^3)
        // = x (-h^4 * r^-3 + 4h - 3 r) / (2h^3)
        // = x (-h/2 * r^-3 + 2/h^2 - 3/(2h^3) r)
        // = x (-3/(2h^3) r + 2/h^2 - h/2 r^-3)
        auto r32 = sqNorm.array();
        auto comm = ((-3.0/2.0/h3) * norm.array() - (m_h/2.0)*norm.array().pow(-3.0)).eval();
        comm = comm.unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
        comm = m_A*2.0/h2 + m_A*comm.array();
        gradientResult->resize(rs.rows(), rs.cols());
        gradientResult->col(0) = rs.col(0).array() * comm.array();
        gradientResult->col(1) = rs.col(1).array() * comm.array();
        //*gradientResult = gradientResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
    if(laplacianResult != nullptr){
        // 
        //   d/dx  x(-3/(2h^3) * r                    - h/2 * r^-3               + 2/h^2)
        // = d/dx (x(-3/(2h^3) * (x^2 + y^2)^.5       - h/2 * (x^2 + y^2)^-1.5   + 2/h^2))
        // = d/dx -3/(2h^3) * x * r                   - h/2 * x * r^-3           + 2/h^2 x
        // = -3/(2h^3) * (2 x^2 + y^2)/r              - h/2 * (r^2 - 3x^2)r^-5   + 2/h^2
        // = -3/(2h^3) * (x^2 + r^2)/r                - h/2 * (r^2 - 3x^2)r^-5   + 2/h^2
        // = -3/(2h^3) * (x^2/r + r)                  - h/2 * (y^2 - 2x^2)(x^2 + y^2)^-2.5 + 2/h^2
        // = -3/(2h^3) * (2 x^2 + y^2)/(x^2 + y^2)^.5 - h/2 * (y^2 - 2x^2)(x^2 + y^2)^-2.5 + 2/h^2
        // = -3/(2h^3) * (x^2 + r^2)/r - h/2 (r^2 - 3x^2)r^-5 + 2/h^2
        // = -3/(2h^3)/r * (x^2 + r^2) - h/2 * r^-5 * (r^2 - 3x^2) + 2/h^2
        //
        // 
        // 
        //
        // laplacian :
        // = -3/(2h^3)/r * (x^2 + r^2) - h/2 * r^-5 * (r^2 - 3x^2) + 2/h^2
        //   -3/(2h^3)/r * (y^2 + r^2) - h/2 * r^-5 * (r^2 - 3y^2) + 2/h^2
        // = -3/(2h^3)/r * (3 r^2) - h/2 * r^-5 * (2*r^2 - 3 r^2) + 4/h^2
        // = -9/(2h^3) * r - h/2 * r^-5 * (-r^2) + 4/h^2
        // = -9/(2h^3) * r + h/2 * r^-3 + 4/h^2

        // = (h^4 + 8 h (x^2 + y^2)^(3/2) - 9 (x^2 + y^2)^2)/(2 h^3 (x^2 + y^2)^(3/2))
        // according to the paper: 45/pi/h^6 (h - r)
        *laplacianResult = (45.0/M_PI)/std::pow(m_h, 6)*(m_h - norm.array());
        //*laplacianResult = laplacianResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
}


} // Physics
} // d2
} // GooBalls
