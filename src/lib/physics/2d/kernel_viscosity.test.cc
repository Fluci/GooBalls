#include "kernel_viscosity.hpp"

#include "kernel_test.hpp"

#include <boost/test/unit_test.hpp>

using namespace GooBalls;
using namespace d2;
using namespace Physics;

namespace utf = boost::unit_test;

typedef Viscosity KKernel;

BOOST_AUTO_TEST_CASE(viscosity_zero_border, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorder(k, 100);
}
// TODO: Viscosity kernel doesn't work
/*
BOOST_AUTO_TEST_CASE(viscosity_normalization, *utf::tolerance(0.0001)){
    KKernel k;
    testNormalization(k, 100);
}
*/
BOOST_AUTO_TEST_CASE(viscosity_monotonicity, *utf::tolerance(0.0001)){
    KKernel k;
    testMonotonicity(k, 10);
}

BOOST_AUTO_TEST_CASE(viscosity_radial_symmetric){
    KKernel k;
    testRadialSymmetry(k, 100);
}
/*
BOOST_AUTO_TEST_CASE(viscosity_grad_finite_diff, *utf::tolerance(0.01)){
    KKernel k;
    testGradientFiniteDifference(k, 100);
}

BOOST_AUTO_TEST_CASE(viscosity_lap_finite_diff, *utf::tolerance(0.00001)){
    KKernel k;
    testLaplacianFiniteDifference(k, 100);
}
*/
