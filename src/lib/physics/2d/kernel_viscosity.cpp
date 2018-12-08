#include "kernel_viscosity.hpp"


namespace GooBalls {
namespace d2 {
namespace Physics {


void Viscosity::setH(Float h) {
    m_h = h;
    // only very rough
    // TODO: get proper formula
    m_A1d = 45.0 / (M_PI * std::pow(h, 6));
    m_A2d = 45.0 / (M_PI * std::pow(h, 6));
}

/// 15.0/(2*pi*h^3) * (-r^3 / (2h^3) + r^2 / h^2 + h / (2r + epsilon) - 1)
///
void Viscosity::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Float epsilon = 0.000001;
    Float A = m_A2d;
    Coordinates1d r2 = rs.rowwise().squaredNorm();
    assert(r2.maxCoeff() <= m_h*m_h*1.01);
    Coordinates1d r1 = r2.array().sqrt();
    Coordinates1d diffH = m_h*m_h - r2.array();
    Coordinates1d diffH2 = diffH.array() * diffH.array();
    assert(diffH.rows() == rs.rows());
    assert(diffH2.rows() == rs.rows());
    Float h2 = m_h*m_h;
    Float h3 = m_h*h2;
    if(wResult != nullptr){
        wResult->resize(rs.rows(), 1);
        auto r3 = r1.array().pow(3.0);
        *wResult = A * (-r3 / (2*h3) + r2.array() / h2 + (m_h / 2.0) / (r2.array() + epsilon) - 1);
    }
    if(gradientResult != nullptr){
        Coordinates1d comm = (-3.0/2.0/h3) * r1.array() - (m_h/2.0)/(r1.array().pow(3.0) + epsilon);
        comm = comm.unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
        comm = A*2.0/h2 + A*comm.array();
        gradientResult->resize(rs.rows(), rs.cols());
        gradientResult->col(0) = rs.col(0).array() * comm.array();
        gradientResult->col(1) = rs.col(1).array() * comm.array();
        //*gradientResult = gradientResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
    if(laplacianResult != nullptr){
        // according to the paper: 45/pi/h^6 (h - r)
        *laplacianResult = A * (m_h - r1.array());
        //*laplacianResult = laplacianResult->unaryExpr([](auto v){return std::isfinite(v) ? v : 0.0;});
    }
}

void Viscosity::compute1d(
        const Coordinates1d& squaredNorm,
        Coordinates1d* wResult,
        Coordinates1d* gradientResult,
        Coordinates1d* laplacianResult) const {
    // NOT IMPLEMENTED
    throw "Not implemented";
}


} // Physics
} // d2
} // GooBalls
