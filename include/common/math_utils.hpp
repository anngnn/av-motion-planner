#ifndef MATH_UTILS_HPP_
#define MATH_UTILS_HPP_
// Shared math helpers used across planning modules.
#include <cmath>
#include "common/types.hpp"

constexpr double kPi = 3.14159265358979323846;

// Straight-line distance between two world-frame points
inline double eucl_dist(const Point& p1, const Point& p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::hypot(dx, dy);
}

// Wrap an angle to [-pi, pi] so it represents the shortest turn.
// A raw heading error (desired - current) can wind far outside this range because
// headings accumulate freely; without wrapping the car would turn the long way around.
// Shifting by full circles (2*pi) never changes the direction an angle points.
inline double normalize_angle(double angle)
{
    while (angle > kPi)
    {
        angle -= 2*kPi;
    }
    while (angle < -kPi)
    {
        angle += 2*kPi;
    }
    return angle;
}

// Dot product of 2D vectors (ax, ay) and (bx, by): multiply matching components,
// add them. When one vector is a unit vector, the result is the length of the
// other's projection onto it (how much of it points that way).
inline double dot(double ax, double ay, double bx, double by)
{
    return ax*bx + ay*by;
}

#endif // MATH_UTILS_HPP_