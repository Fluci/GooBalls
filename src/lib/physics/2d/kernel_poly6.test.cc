#include "kernel_poly6.hpp"

#include "kernel_test.hpp"

#include <boost/test/unit_test.hpp>

using namespace GooBalls;
using namespace d2;
using namespace Physics;

namespace utf = boost::unit_test;

typedef Poly6 KKernel;

BOOST_AUTO_TEST_CASE(poly6_zero_border, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorder(k, 100);
}

BOOST_AUTO_TEST_CASE(poly6_normalization, *utf::tolerance(0.0001)){
    KKernel k;
    testNormalization(k, 100);
}

BOOST_AUTO_TEST_CASE(poly6_monotonicity, *utf::tolerance(0.0001)){
    KKernel k;
    testMonotonicity(k, 10);
}

BOOST_AUTO_TEST_CASE(poly6_radial_symmetric){
    KKernel k;
    testRadialSymmetry(k, 100);
}

BOOST_AUTO_TEST_CASE(poly6_grad_finite_diff, *utf::tolerance(0.001)){
    KKernel k;
    testGradientFiniteDifference(k, 100);
}

BOOST_AUTO_TEST_CASE(poly6_lap_finite_diff, *utf::tolerance(0.00001)){
    KKernel k;
    testLaplacianFiniteDifference(k, 100);
}

