#include "kernel_debrun_spiky.hpp"

#include <iostream>

namespace GooBalls {
namespace d2 {
namespace Physics {

void DebrunSpiky::setH(Float h) {
    m_h = h;
    m_A1d = 2/std::pow(m_h, 4);
    m_A2d = 10.0/M_PI/std::pow(h, 5);;
}

void DebrunSpiky::compute1d(
        const Coordinates1d& r2,
        Coordinates1d* wResult,
        Coordinates1d* gradientResult,
        Coordinates1d* laplacianResult) const {
    Float h = m_h;
    Float A = m_A1d;
    assert(r2.maxCoeff() <= h*h*1.01);
    Coordinates1d r1 = r2.array().sqrt();
    Coordinates1d diffH = h - r1.array();
    Coordinates1d diffH2 = diffH.array() * diffH.array();
    if(wResult != nullptr){
        *wResult = A * diffH2.array()*diffH.array();
    }
    if(gradientResult != nullptr){
        *gradientResult = (-3*A) * diffH2;
    }
    if(laplacianResult != nullptr){
        *laplacianResult = (6.0*A)*(h - r1.array());
    }
}

void DebrunSpiky::compute(
        const Coordinates2d& rs,
        Coordinates1d* wResult,
        Coordinates2d* gradientResult,
        Coordinates1d* laplacianResult) const {
    Float h = m_h;
    Float A = m_A2d;
    Float epsilon = 0.0001;
    Coordinates1d r2 = rs.rowwise().squaredNorm();
    assert(r2.maxCoeff() <= h*h*1.01);
    Coordinates1d r1 = r2.array().sqrt();
    Coordinates1d diffH = h - r1.array();
    Coordinates1d diffH2 = diffH.array() * diffH.array();
    auto r1Inv = 1.0/(r1.array() + h*epsilon);
    if(wResult != nullptr){
        *wResult = A * diffH2.array()*diffH.array();
    }
    if(gradientResult != nullptr){
        auto comm = (-3*A) * diffH2;
        *gradientResult = rs.array().colwise() * r1Inv;
        *gradientResult = gradientResult->array().colwise() * comm.array();
        /*auto comm = (-3*A) * diffH2.array() * r1Inv.array();
        gradientResult->resize(rs.rows(), rs.cols());
        gradientResult->col(0) = comm.array() * rs.col(0).array();
        gradientResult->col(1) = comm.array() * rs.col(1).array();
        *gradientResult = gradientResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
*/
    }
    if(laplacianResult != nullptr){
        Float h2 = h*h;
        *laplacianResult = -3*A * (h2 * r1Inv + 3.0 * r1.array() - 4 * h);
        /*laplacianResult->resize(rs.rows(), 1);
        *laplacianResult = (h*h * r1Inv) + (3.0*r1.array()) ;
        *laplacianResult = laplacianResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
        *laplacianResult = (-3.0*A)*(laplacianResult->array() - 4.0*h);
        */
    }
}


} // Physics
} // d2
} // GooBalls
