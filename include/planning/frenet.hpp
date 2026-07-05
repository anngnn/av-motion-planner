#ifndef FRENET_HPP_
#define FRENET_HPP_

#include <vector>
#include "common/types.hpp"

// A fixed anchor point that defines the reference line itself, stored in world
// coordinates. These are the "mile markers": each has a real (x, y), the path's
// heading there, and its arc length s. The RefLine (vector of these) is built once
// from the A* path and is what we use to convert between world and Frenet frames.
struct RefPoint
{
    Pose world_pos; // x, y in world frame; theta = path tangent (perpendicular = d axis)
    double s;       // cumulative arc length from the first point (s = 0 at the start)
};

// A position expressed in the road-relative Frenet frame, not world coordinates.
// Its meaning depends entirely on the reference line: s = how far along, d = how far
// to the side. Used for the car, obstacles, and candidate trajectory points.
struct FrenetPoint
{
    double s;  // distance ALONG the reference line (monotonically increasing forward)
    double d;  // lateral offset FROM the line (signed: + one side, - the other)
};

// The reference line: an ordered sequence of RefPoints with strictly increasing s.
using RefLine = std::vector<RefPoint>;

RefLine path_to_refline(const Path & path);

#endif // FRENET_HPP_