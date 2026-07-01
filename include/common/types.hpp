#ifndef COMMON_TYPES_HPP_
#define COMMON_TYPES_HPP_

#include <vector>

// A location in 2D space — used for obstacle positions and road waypoints
// No direction; just where something is
struct Point
{
    double x;  // meters, world frame
    double y;  // meters, world frame
};

// A location + the direction the vehicle is facing
// theta is needed to decompose velocity into x/y components in the bicycle model
struct Pose
{
    double x;      // meters, world frame
    double y;      // meters, world frame
    double theta;  // heading in radians (0 = facing +x axis, pi/2 = facing +y axis)
};

// A fully specified point on a motion plan: where, which way, how fast, and when
// Frenet planner outputs these; the vehicle executes them one by one
struct TrajectoryPoint
{
    double x;      // meters, world frame
    double y;      // meters, world frame
    double theta;  // heading in radians
    double v;      // speed in m/s at this point
    double t;      // seconds from now when the car should reach this point
};

// A* outputs a Path — ordered waypoints only, no timing or speed
using Path = std::vector<Point>;

// Frenet planner outputs a Trajectory — fully specified with speed + timestamps
// The vehicle needs both to know how fast to move and when to be where
using Trajectory = std::vector<TrajectoryPoint>;

#endif  // COMMON_TYPES_HPP_
