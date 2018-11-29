#include "neighborhood_brute_force.hpp"
#include <iostream>

namespace GooBalls {

namespace Spatial {


void NeighborhoodBruteForce::inRange(const Coordinates2d& points, Float h){
    inRange(points, points, h);
}

void NeighborhoodBruteForce::inRange(const Coordinates2d& query, const Coordinates2d& target_points, Float h){
    // reset members
    const int PN = query.rows();
    resetIndexes(PN);
    // find output
    Float h2 = h*h;
    for(int i = 0; i < PN; ++i){
        auto diff = target_points.rowwise() - query.row(i);
        auto squared = diff.array() * diff.array();
        Coordinates1d sqDists = squared.rowwise().sum();
        for(int j = 0; j < target_points.rows(); ++j){
            if(sqDists[j] <= h2){
                m_indexes[i].push_back(j);
            }
        }
    }
}


} // Spatial

} // GooBalls
