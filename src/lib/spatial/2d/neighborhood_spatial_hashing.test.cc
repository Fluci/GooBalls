#include "neighborhood_brute_force.hpp"
#include "neighborhood_spatial_hashing.hpp"

#include <algorithm>

#include <boost/test/unit_test.hpp>

using namespace GooBalls;
using namespace Spatial;

BOOST_AUTO_TEST_CASE(spatial_hashing_self_inRange){
    NeighborhoodSpatialHashing test;

    Coordinates2d pts(10,2);
    pts.setZero(10, 2);
    pts(0,0) = -1.0;
    pts(1,0) = -0.8;
    pts(2,0) = -0.6;
    pts(3,0) = -0.4;
    pts(4,0) = -0.2;
    pts(5,0) = 0.0;
    pts(6,0) = 0.2;
    pts(7,0) = 0.4;
    pts(8,0) = 0.6;
    pts(9,0) = 0.8;

    CoordinatePrecision h = 0.25;

    test.inRange(pts, h);

    std::vector<std::vector<int>> expected(pts.rows());
    expected[0].push_back(0);
    expected[0].push_back(1);

    expected[1].push_back(0);
    expected[1].push_back(1);
    expected[1].push_back(2);

    expected[2].push_back(1);
    expected[2].push_back(2);
    expected[2].push_back(3);

    expected[3].push_back(2);
    expected[3].push_back(3);
    expected[3].push_back(4);

    expected[4].push_back(3);
    expected[4].push_back(4);
    expected[4].push_back(5);

    expected[5].push_back(4);
    expected[5].push_back(5);
    expected[5].push_back(6);

    expected[6].push_back(5);
    expected[6].push_back(6);
    expected[6].push_back(7);

    expected[7].push_back(6);
    expected[7].push_back(7);
    expected[7].push_back(8);

    expected[8].push_back(7);
    expected[8].push_back(8);
    expected[8].push_back(9);

    expected[9].push_back(8);
    expected[9].push_back(9);

    auto& indexes = test.indexes();
    for(auto& e : indexes){
        std::sort(e.begin(), e.end());
    }
    BOOST_CHECK_EQUAL(expected.size(), indexes.size());
    for(size_t i = 0; i < expected.size(); ++i){
        BOOST_CHECK_EQUAL_COLLECTIONS(expected[i].begin(), expected[i].end(), indexes[i].begin(), indexes[i].end());
    }
}

BOOST_AUTO_TEST_CASE(spatial_hashing_self_inrange_random) {
    NeighborhoodBruteForce truth;
    NeighborhoodSpatialHashing test;

    Coordinates2d testPoints;
    testPoints.setRandom(100, 2);

    CoordinatePrecision h = 0.05;

    truth.inRange(testPoints, h);
    test.inRange(testPoints, h);

    BOOST_CHECK_EQUAL(truth.indexes().size(), test.indexes().size()); 
    for(size_t i = 0; i < truth.indexes().size(); ++i){
        std::sort(truth.indexes()[i].begin(), truth.indexes()[i].end());
        std::sort(test.indexes()[i].begin(), test.indexes()[i].end());
	BOOST_CHECK_EQUAL_COLLECTIONS(truth.indexes()[i].begin(), truth.indexes()[i].end(), test.indexes()[i].begin(), test.indexes()[i].end());
    } 
}

BOOST_AUTO_TEST_CASE(spatial_hashing_query_target_inrange) {
    NeighborhoodBruteForce truth;
    NeighborhoodSpatialHashing test;

    Coordinates2d queryPoints;
    queryPoints.setRandom(100, 2);

    Coordinates2d targetPoints;
    targetPoints.setRandom(100, 2);

    CoordinatePrecision h = 0.05;

    truth.inRange(queryPoints, targetPoints, h);
    test.inRange(queryPoints, targetPoints, h);

    BOOST_CHECK_EQUAL(truth.indexes().size(), test.indexes().size()); 
    for(size_t i = 0; i < truth.indexes().size(); ++i){
        std::sort(truth.indexes()[i].begin(), truth.indexes()[i].end());
        std::sort(test.indexes()[i].begin(), test.indexes()[i].end());
	BOOST_CHECK_EQUAL_COLLECTIONS(truth.indexes()[i].begin(), truth.indexes()[i].end(), test.indexes()[i].begin(), test.indexes()[i].end());
    } 
}

