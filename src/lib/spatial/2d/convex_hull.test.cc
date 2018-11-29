#include "convex_hull.hpp"

#include <boost/test/unit_test.hpp>

using namespace GooBalls;
using namespace Spatial;
using namespace impl;

BOOST_AUTO_TEST_CASE(convex_hull_angles){
	Point p0;
	p0.x = 1;
	p0.y = 1;

	std::vector<Point> ps(3);

	// -45°
	ps[0].x = 2;
	ps[0].y = 0;

	// 0°
	ps[1].x = 2;
	ps[1].y = 1;
	
	ps[2].x = 2;
	ps[2].y = 2;

	computeAngles(ps.begin(), ps.end(), p0);

	BOOST_CHECK_EQUAL(-45.0/180*M_PI, ps[0].angle);
	BOOST_CHECK_EQUAL(0.0/180*M_PI, ps[1].angle);
	BOOST_CHECK_EQUAL(45.0/180*M_PI, ps[2].angle);
}

BOOST_AUTO_TEST_CASE(convex_hull_turn_direction){
	Point p0;
	p0.x = 1;
	p0.y = 1;
	Point p1;
	p1.x = 2;
	p1.y = 1;
	Point p2;
	p2.x = 1;
	p2.y = 2;
	Point p3;
	p3.x = 3;
	p3.y = 1;

	BOOST_CHECK_EQUAL(Left, turnDirection(p0, p1, p2));
    BOOST_CHECK_EQUAL(Right, turnDirection(p0, p2, p1));
	BOOST_CHECK_EQUAL(None, turnDirection(p0, p1, p3));
}

BOOST_AUTO_TEST_CASE(convex_hull_test1){
	Coordinates2d pointSet(5, 2);
	// lower left corner
	pointSet(0,0) = -1;
	pointSet(0,1) = -1;
	// upper left corner
	pointSet(1,0) = -1;
	pointSet(1,1) = 1;
	// center
	pointSet(2,0) = 0.5;
	pointSet(2,1) = -0.1;
	// lower right corner
	pointSet(3,0) = 1;
	pointSet(3,1) = -1;
	// upper right corner
	pointSet(4,0) = 1;
	pointSet(4,1) = 1;

    pointSet = convex_hull(pointSet);

    BOOST_CHECK_EQUAL(4, pointSet.rows());
    // To be precise: this uses knowledge of the implementation, according to the contract, we could start with any point of the convex hull
    BOOST_CHECK_EQUAL(-1, pointSet(0,0));
    BOOST_CHECK_EQUAL(-1, pointSet(0,1));
    BOOST_CHECK_EQUAL(1, pointSet(1,0));
    BOOST_CHECK_EQUAL(-1, pointSet(1,1));
    BOOST_CHECK_EQUAL(1, pointSet(2,0));
    BOOST_CHECK_EQUAL(1, pointSet(2,1));
    BOOST_CHECK_EQUAL(-1, pointSet(3,0));
    BOOST_CHECK_EQUAL(1, pointSet(3,1));
}
