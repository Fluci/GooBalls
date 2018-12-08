#include "kernel_poly6.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {


void Poly6::setH(Float h) {
    m_h = h;
    //m_A = 4.0/M_PI/std::pow(h, 8);
    //m_A = 16.0/M_PI/std::pow(h, 6);
    m_A = 4.0/M_PI/std::pow(h, 8);
}

template <typename DerivedIn, typename DerivedOut1, typename DerivedOut2, typename DerivedOut3>
void computePoly6(
        const Eigen::MatrixBase<DerivedIn>& r2,
        Eigen::MatrixBase<DerivedOut1>* wResult,
        Eigen::MatrixBase<DerivedOut2>* gradientResult,
        Eigen::MatrixBase<DerivedOut3>* laplacianResult,
        Float h,
        Float A) {
    assert(r2.maxCoeff() <= h*h*1.01);
    Coordinates1d sqDiffH = h*h - r2.array();
    Coordinates1d sqDiffH2 = sqDiffH.array() * sqDiffH.array();
    assert(sqDiffH.rows() == r2.rows());
    assert(sqDiffH2.rows() == r2.rows());
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
        gradientResult->derived().resize(r2.rows(), Eigen::NoChange);
        auto r1 = r2.array().sqrt();
        gradientResult->col(0) = (-6.0 * A) * sqDiffH2.array() * r1.array();
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
        //-6*A*(h^4 - 6h^2 r^2 + 5r^4)
        Float h2 = h*h;
        Float h4 = h2*h2;
        auto r4 = r2.array()*r2.array();
        *laplacianResult = (-6.0 * A) * (h4 - 6*h2 * r2.array() + 5* r4);
    }
}

void Poly6::compute1d(
        const Coordinates1d& squaredNorm,
        Coordinates1d* wResult,
        Coordinates1d* gradientResult,
        Coordinates1d* laplacianResult) const {
    computePoly6(squaredNorm, wResult, gradientResult, laplacianResult, m_h, 35.0/32.0/std::pow(m_h, 7));
}

void Poly6::compute(
        const Coordinates2d& rs, 
        Coordinates1d* wResult, 
        Coordinates2d* gradientResult, 
        Coordinates1d* laplacianResult) const {
    Coordinates1d r2 = rs.rowwise().squaredNorm();
    computePoly6(r2, wResult, gradientResult, laplacianResult, m_h, m_A);
    if(gradientResult != nullptr){
        Coordinates2d rsN = rs.array().colwise() / (r2.array().sqrt() + m_h*0.000001);
        gradientResult->col(1) = rsN.col(1).array() * gradientResult->col(0).array();
        gradientResult->col(0) = rsN.col(0).array() * gradientResult->col(0).array();
    }
    if(laplacianResult != nullptr){
    	// -6*A * (h^2 - r^2) * (2.0 * h^2 - 6.0*r^2)
    	Float h2 = m_h * m_h;
        *laplacianResult = (-6.0 * m_A) * (h2 - r2.array()) * (2.0*h2 - 6.0*r2.array());
        //auto r = r2.array().sqrt();
    	//*laplacianResult = laplacianResult->array() / r.array();
    }
}


} // Physics
} // d2
} // GooBalls
