#pragma once

#include "types.hpp"

namespace GooBalls {

namespace Spatial {

/// Finds the convex hull of a set of points.
/// The returned matrix only consists of the points making up the convex hull
/// The points are ordered in counterclockwise order
Coordinates2d convex_hull(Coordinates2d pointSet);

namespace impl {

struct Point {
    Float x;
    Float y;
    Float angle;
};

/// Helper that finds angles between x-axis and given points relative to p0
template <typename Iterator>
void computeAngles(Iterator begin, Iterator end, const Point p0){
    std::for_each(begin, end, [=](auto& p){p.angle = std::atan2(p.y - p0.y, p.x - p0.x);});
}

enum Direction {
    Left,
    None,
    Right
};
Direction turnDirection(const Point a, const Point b, const Point c);


}

} // Spatial

} // GooBalls

