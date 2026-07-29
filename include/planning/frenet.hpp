#ifndef FRENET_HPP_
#define FRENET_HPP_

#include <vector>
#include "common/types.hpp"

// ---------------------------------------------------------------------------
// Reference line: the smooth centerline the Frenet frame is defined against.
// ---------------------------------------------------------------------------

// A fixed anchor point that defines the reference line itself, stored in world
// coordinates. These are the "mile markers": each has a real (x, y), the path's
// heading there, and its arc length s. The RefLine (vector of these) is built once
// from the A* path and is what we use to convert between world and Frenet frames.
struct RefPoint
{
    Pose world_pos; // x, y in world frame; theta = path tangent (perpendicular = d axis)
    double s;       // cumulative arc length from the first point (s = 0 at the start)
};

// The reference line: an ordered sequence of RefPoints with strictly increasing s.
using RefLine = std::vector<RefPoint>;

// Build a reference line from an A* path: fills in each point's heading (path
// tangent) and cumulative arc length s. Returns empty if the path has < 2 points.
RefLine path_to_ref_line(const Path & path);

// ---------------------------------------------------------------------------
// Frenet coordinates and the world <-> Frenet transforms.
// ---------------------------------------------------------------------------

// A position expressed in the road-relative Frenet frame, not world coordinates.
// Its meaning depends entirely on the reference line: s = how far along, d = how far
// to the side. Used for the car, obstacles, and candidate trajectory points.
struct FrenetPoint
{
    double s;  // distance ALONG the reference line (monotonically increasing forward)
    double d;  // lateral offset FROM the line (signed: + one side, - the other)
};

// World (x, y) -> Frenet (s, d): projects the point onto the reference line.
// s = arc length of the nearest point plus how far along past it, d = signed offset.
FrenetPoint to_frenet(const Point & world_p, const RefLine & rline);

// Frenet (s, d) -> world (x, y): the inverse. Interpolates a base point at arc
// length s along the line, then steps d perpendicular to the line's heading.
Point from_frenet(const FrenetPoint & fp, const RefLine & rline);

// ---------------------------------------------------------------------------
// Candidate trajectory generation.
// ---------------------------------------------------------------------------

// The car's full motion state in the Frenet frame: position, velocity, and
// acceleration on both axes. Serves as the start boundary conditions for the
// polynomials when generating candidate trajectories.
struct FrenetState
{
    double s, s_dot, s_ddot;  // longitudinal: position, velocity, acceleration
    double d, d_dot, d_ddot;  // lateral:      position, velocity, acceleration
};

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

// Tuning knobs for the Frenet candidate sampler. All SI units (meters, seconds, m/s).
// The generator sweeps three axes: lateral offset d, maneuver horizon T, target speed.
struct FrenetConfig
{
    // lateral offset sweep: sample d in [-max_road_width, +max_road_width]
    double max_road_width = 3.0;   // how far to each side of the reference line
    int    num_d_samples  = 7;     // how many lateral offsets to try

    // time-horizon sweep: sample T in [min_t, max_t]
    double min_t          = 2.0;   // shortest maneuver duration
    double max_t          = 5.0;   // longest maneuver duration
    int    num_t_samples  = 4;     // how many horizons to try

    // speed sweep: sample target_speed +/- speed_range, centered on the goal speed
    double target_speed      = 5.0;  // desired cruising speed
    double speed_range       = 1.0;  // half-width of the speed sweep
    int    num_speed_samples = 3;    // how many speeds to try

    double dt = 0.1;  // time step when sampling points along each trajectory

    double car_radius = 1.5;  // obstacles inflated by this; extra berth covers loose pure-pursuit tracking
};

// Generate all candidate trajectories by sweeping lateral offset d, horizon T, and
// target speed. Each candidate starts from `start` and is sampled every config.dt.
std::vector<FrenetTrajectory> generate_frenet_trajectories(
    const FrenetState & start, const RefLine & rline, const FrenetConfig & config);

// Weights for scoring a candidate trajectory. cost = sum of each penalty times its
// weight; lower cost = better. Tune the RATIO between these to change the car's
// personality (e.g. raise w_jerk to prefer smoother, gentler trajectories).
struct CostWeights
{
    double w_jerk        = 1.0;  // penalize rough motion (jerk) -> comfort
    double w_offcenter   = 1.0;  // penalize straying from the lane center (large |d|)
    double w_speed_error = 1.0;  // penalize ending far from the target speed
};

double compute_cost(const FrenetTrajectory & traj, const CostWeights & weights, const FrenetConfig & config);

// True if the trajectory passes too close to any obstacle. Each obstacle is inflated
// by car_radius so the car can be treated as a single point along the trajectory.
bool is_collision(const std::vector<Obstacle> & obstacles, const FrenetTrajectory & traj, double car_radius);

#endif // FRENET_HPP_