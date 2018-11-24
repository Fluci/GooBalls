#include "kernel_debrun_spiky.hpp"

#include "kernel_test.hpp"

#include <boost/test/unit_test.hpp>

using namespace GooBalls;
using namespace d2;
using namespace Physics;

namespace utf = boost::unit_test;

typedef DebrunSpiky KKernel;

BOOST_AUTO_TEST_CASE(debrun_spiky_zero_border, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorder(k, 100);
}

BOOST_AUTO_TEST_CASE(debrun_spiky_zero_border_gradient, *utf::tolerance(0.0001)){
    KKernel k;
    testZeroBorderGradient(k, 100);
}

BOOST_AUTO_TEST_CASE(debrun_spiky_normalization, *utf::tolerance(0.0001)){
    KKernel k;
    testNormalization(k, 100);
}


BOOST_AUTO_TEST_CASE(debrun_spiky_monotonicity, *utf::tolerance(0.0001)){
    KKernel k;
    testMonotonicity(k, 100);
}

BOOST_AUTO_TEST_CASE(debrun_spiky_non_negativity){
    KKernel k;
    testNonNegativity(k, 100);
}

BOOST_AUTO_TEST_CASE(debrun_spiky_radial_symmetric){
    KKernel k;
    testRadialSymmetry(k, 100);
}

BOOST_AUTO_TEST_CASE(debrun_spiky_grad_finite_diff, *utf::tolerance(0.01)){
    KKernel k;
    testGradientFiniteDifference(k, 100);
}

BOOST_AUTO_TEST_CASE(debrun_spiky_lap_finite_diff, *utf::tolerance(0.00001)){
    KKernel k;
    testLaplacianFiniteDifference(k, 100);
}

