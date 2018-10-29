#include "hello.hpp"

#include <boost/test/unit_test.hpp>

using namespace GooBalls;

BOOST_AUTO_TEST_CASE(demo_test_1) {
  BOOST_CHECK_EQUAL(17, hello()); 
}
