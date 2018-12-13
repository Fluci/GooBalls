#include "kernel_poly6.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {


void Poly6::setH(Float h) {
    m_h = h;
    //m_A = 4.0/M_PI/std::pow(h, 8);
    //m_A = 16.0/M_PI/std::pow(h, 6);
    m_A1d = 35.0/32.0/std::pow(m_h, 7);
    m_A2d = 4.0/M_PI/std::pow(h, 8);
}

void Poly6::compute1d(
        const Coordinates1d& r2,
        Coordinates1d* wResult,
        Coordinates1d* gradientResult,
        Coordinates1d* laplacianResult) const {
    Float h = m_h;
    Float A = m_A1d;
    Float h2 = h*h;
    assert(r2.maxCoeff() <= h*h*1.01);
    Coordinates1d sqDiffH = h2 - r2.array();
    Coordinates1d sqDiffH2 = sqDiffH.array() * sqDiffH.array();
    assert(sqDiffH.rows() == r2.rows());
    assert(sqDiffH2.rows() == r2.rows());
    if(wResult != nullptr){
        *wResult = A * sqDiffH2.array() * sqDiffH.array();
    }
    if(gradientResult != nullptr){
        gradientResult->derived().resize(r2.rows(), Eigen::NoChange);
        auto r1 = r2.array().sqrt();
        gradientResult->col(0) = (-6.0 * A) * sqDiffH2.array() * r1.array();
    }
    if(laplacianResult != nullptr){
        Float h4 = h2*h2;
        auto r4 = r2.array()*r2.array();
        *laplacianResult = (-6.0 * A) * (h4 - 6*h2 * r2.array() + 5* r4);
    }
}

void Poly6::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Coordinates1d r2 = rs.rowwise().squaredNorm();
    Float h = m_h;
    Float A = m_A2d;
    Float epsilon = m_epsilon;
    Float h2 = h * h;
    Coordinates1d sqDiffH = h2 - r2.array();
    Coordinates1d sqDiffH2 = sqDiffH.array() * sqDiffH.array();
    if(wResult != nullptr){
        *wResult = A * sqDiffH2.array() * sqDiffH.array();
    }
    if(gradientResult != nullptr){
        gradientResult->resize(r2.rows(), Eigen::NoChange);
        auto r1 = r2.array().sqrt();
        gradientResult->col(0) = (-6.0 * A) * sqDiffH2.array() * r1.array();
        auto rsN = rs.array().colwise() / (r2.array().sqrt() + h*epsilon);
        *gradientResult = rsN.array().colwise() * gradientResult->col(0).array();
    }
    if(laplacianResult != nullptr){
    	// -6*A * (h^2 - r^2) * (2.0 * h^2 - 6.0*r^2)
        *laplacianResult = (-6.0 * A) * (h2 - r2.array()) * (2.0*h2 - 6.0*r2.array());
        //auto r = r2.array().sqrt();
    	//*laplacianResult = laplacianResult->array() / r.array();
    }
}


} // Physics
} // d2
} // GooBalls
