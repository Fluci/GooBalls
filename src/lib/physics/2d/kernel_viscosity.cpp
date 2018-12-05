#include "kernel_viscosity.hpp"


namespace GooBalls {
namespace d2 {
namespace Physics {


void Viscosity::setH(Float h) {
    m_h = h;
    // only very rough
    // TODO: get proper formula
    m_A = 45.0 / (M_PI * std::pow(h, 6));
}

template <typename DerivedIn, typename DerivedOut1, typename DerivedOut2, typename DerivedOut3>
void computeViscosity(
        const Eigen::MatrixBase<DerivedIn>& sqNorm,
        Eigen::MatrixBase<DerivedOut1>* wResult,
        Eigen::MatrixBase<DerivedOut2>* gradientResult,
        Eigen::MatrixBase<DerivedOut3>* laplacianResult,
        Float h,
        Float A) {
    Float epsilon = 0.0000001;
    assert(sqNorm.maxCoeff() <= h*h*1.01);
    Coordinates1d norm = sqNorm.array().sqrt();
    Coordinates1d diffH = h*h - sqNorm.array();
    Coordinates1d diffH2 = diffH.array() * diffH.array();
    assert(diffH.rows() == sqNorm.rows());
    assert(diffH2.rows() == sqNorm.rows());
    Float h2 = h*h;
    Float h3 = h*h2;
    if(wResult != nullptr){
        // 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r) - 1)
        // ((-(x^2 + y^2)^(3/2) / (2h^3) + (x^2 + y^2) / h^2 + h/2 * (x^2 + y^2)^(-1/2)) - 1))
        auto r3 = norm.array().pow(3.0);
        *wResult = A * (-r3 / (2*h3) + sqNorm.array() / h2 + (h / 2.0) / (norm.array() + epsilon) - 1);
    }
    if(gradientResult != nullptr){
        // d/dx (-r^3 / (2h^3)          + r^2 / h^2         + h/2 * r^-1               - 1)
        // d/dx (-(x^2 + y^2)^1.5/(2h^3)+ (x^2 + y^2) / h^2 + h/2 * (x^2 + y^2)^(-1/2) - 1)
        //
        // = x (-h^4 * (x^2 + y^2)^-1.5 + 4h - 3(x^2 + y^2)^0.5)/(2h^3)
        // = x (-h^4 * r^-3 + 4h - 3 r) / (2h^3)
        // = x (-h/2 * r^-3 + 2/h^2 - 3/(2h^3) r)
        // = x (-3/(2h^3) r + 2/h^2 - h/2 r^-3)
        // wolfram alpha:
        // gradient of -(x^2 + y^2)^(3/2) / (2*h^3)
        // = (-(3 x sqrt(x^2 + y^2))/(2 h^3), -(3 y sqrt(x^2 + y^2))/(2 h^3))
        // = (-(3 x |r|/(2h^3), -(3 y |r|/2h^3))
        // gradient of (x^2 + y^2) / h^2
        // = ((2 x)/h^2, (2 y)/h^2)
        // gradient of h/(2 sqrt(x^2 + y^2))
        // = (-(h x)/(2 (x^2 + y^2)^(3/2)), -(h y)/(2 (x^2 + y^2)^(3/2)))
        // = (-(h x)/(2 r^3), -(h y)/(2 r^3))
        //
        // total:
        // -3x |r|/(2h^3) + 2 x/h^2 - h x /(2 r^3)
        // x (-3 |r|/(2h^3) + 2/h^2 - h/(2r^3)
        // wolfram alpha:
        // -(h^4 - 4 h r^3 + 3 r^4)/(2 h^3 r^2)
        //
        Coordinates1d comm = -(h2*h2 - 4*h*norm.array()*sqNorm.array() + 3 * sqNorm.array()*sqNorm.array())/(sqNorm.array() + epsilon);
        comm = comm.unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
        gradientResult->derived().resize(sqNorm.rows(), Eigen::NoChange);
        gradientResult->col(0) = A*comm.array();
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
        *laplacianResult = A * (h - norm.array());
        //*laplacianResult = laplacianResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
}

/// 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r + epsilon) - 1)
///
void Viscosity::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Coordinates1d sqNorm = rs.rowwise().squaredNorm();
    computeViscosity(rs.rowwise().squaredNorm(), wResult, gradientResult, laplacianResult, m_h, m_A);
    if(gradientResult != nullptr){
        gradientResult->col(1) = rs.col(1).array() * gradientResult->col(0).array();
        gradientResult->col(0) = rs.col(0).array() * gradientResult->col(0).array();
    }
}


} // Physics
} // d2
} // GooBalls
