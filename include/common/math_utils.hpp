#ifndef MATH_UTILS_HPP_
#define MATH_UTILS_HPP_
#include <cmath>
#include "types.hpp"

// Straight-line distance between two world-frame points
inline double eucl_dist(const Point& p1, const Point& p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::hypot(dx, dy);
}

#endif // MATH_UTILS_HPP_