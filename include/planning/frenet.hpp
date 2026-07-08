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

// Build a reference line from an A* path: fills in each point's heading (path
// tangent) and cumulative arc length s. Returns empty if the path has < 2 points.
RefLine path_to_ref_line(const Path & path);

// World (x, y) -> Frenet (s, d): projects the point onto the reference line.
// s = arc length of the nearest point plus how far along past it, d = signed offset.
FrenetPoint to_frenet(const Point & world_p, const RefLine & rline);

// Frenet (s, d) -> world (x, y): the inverse. Interpolates a base point at arc
// length s along the line, then steps d perpendicular to the line's heading.
Point from_frenet(const FrenetPoint & fp, const RefLine & rline);

// One candidate trajectory the car could follow over the next few seconds.
// Struct-of-arrays: every vector has the same length, and index i is one time step.
// The polynomials fill the Frenet columns; from_frenet fills x_world/y_world.
struct FrenetTrajectory
{
    std::vector<double> t;        // timestamps, 0 .. T

    // longitudinal (along the road) state and its derivatives
    std::vector<double> s;        // arc length
    std::vector<double> s_dot;    // speed
    std::vector<double> s_ddot;   // acceleration
    std::vector<double> s_dddot;  // jerk (for the cost function)

    // lateral (across the road) state and its derivatives
    std::vector<double> d;        // offset from the reference line
    std::vector<double> d_dot;    // lateral velocity
    std::vector<double> d_ddot;   // lateral acceleration
    std::vector<double> d_dddot;  // lateral jerk (for the cost function)

    // world-frame position, converted from (s, d) for drawing and collision
    std::vector<double> x_world;
    std::vector<double> y_world;

    double cost = 0.0;            // filled in when the trajectory is scored
};

#endif // FRENET_HPP_