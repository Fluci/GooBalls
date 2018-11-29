#include "convex_hull.hpp"

#include <vector>
#include <algorithm>

namespace GooBalls {

namespace Spatial {

namespace impl {

Direction turnDirection(const Point a, const Point b, const Point c) {
    Float direction = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    return direction < 0.0 ? Right : (direction > 0.0 ? Left : None);
}

}

using namespace impl;

/// https://de.wikipedia.org/wiki/Graham_Scan
Coordinates2d convex_hull(Coordinates2d pointSet) {
    if(pointSet.rows() <= 0){
        return pointSet;
    }
    std::vector<Point> points(pointSet.rows());
    int minI = 0;
    for(int i = 0; i < points.size(); ++i){
        points[i].x = pointSet(i, 0);
        points[i].y = pointSet(i, 1);
        if(points[i].x <= points[minI].x){
            if(points[i].x < points[minI].x){
                minI = i;
            } else if (points[i].y < points[minI].y) {
                minI = i;
            }
        }
    }
    Point p0 = points[minI];
    points.erase(points.begin() + minI);
    computeAngles(points.begin(), points.end(), p0);
    std::sort(points.begin(), points.end(), [](auto& a, auto& b){return a.angle < b.angle;});
    std::vector<Point> stack;
    stack.push_back(p0);
    stack.push_back(points.front());
    int i = 1;
    while(i < points.size()){
        int j = stack.size()-1;
        auto turnDir = turnDirection(stack[j-1], stack[j], points[i]);
        if(turnDir == Left || stack.size() == 2){
            stack.push_back(points[i]);
            i += 1;
        } else {
            stack.pop_back();
        }
    }
    pointSet.resize(stack.size(), Eigen::NoChange);
    for(int i = 0; i < stack.size(); ++i){
        pointSet(i, 0) = stack[i].x;
        pointSet(i, 1) = stack[i].y;
    }
    return pointSet;
}

} // Spatial

} // GooBalls
