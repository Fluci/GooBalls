#include "kernel_poly_viscosity.hpp"

#include "kernel_test.hpp"

#include <boost/test/unit_test.hpp>

using namespace GooBalls;
using namespace d2;
using namespace Physics;

namespace utf = boost::unit_test;

using KKernel = PolyViscosity;

BOOST_AUTO_TEST_CASE(poly_viscosity_zero_border_1d, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorder1d(k);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_normalization_1d, *utf::tolerance(0.0001)){
    KKernel k;
    testNormalization1d(k, 100000);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_zero_border_gradient_1d, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorderGradient1d(k);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_grad_finite_diff_1d, *utf::tolerance(0.001)){
    KKernel k;
    testGradientFiniteDifference1d(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_lap_grad_finite_diff_1d, *utf::tolerance(0.0001)){
    KKernel k;
    testLaplacianFromGradientFiniteDifferences1d(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_lap_finite_diff_1d, *utf::tolerance(0.00001)){
    KKernel k;
    testLaplacianFiniteDifference1d(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_zero_border, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorder(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_zero_border_gradient, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorderGradient(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_normalization, *utf::tolerance(0.00001)){
    KKernel k;
    testNormalization(k, 100000);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_monotonicity, *utf::tolerance(0.0001)){
    KKernel k;
    testMonotonicity(k, 10);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_radial_symmetric){
    KKernel k;
    testRadialSymmetry(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_grad_finite_diff, *utf::tolerance(0.0005)){
    KKernel k;
    testGradientFiniteDifference(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_lap_grad_finite_diff, *utf::tolerance(0.001)){
    KKernel k;
    testLaplacianFromGradientFiniteDifferences(k, 100);
}

BOOST_AUTO_TEST_CASE(poly_viscosity_lap_finite_diff, *utf::tolerance(0.001)){
    KKernel k;
    testLaplacianFiniteDifference(k, 100);
}
