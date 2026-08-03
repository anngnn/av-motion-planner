#ifndef BEHAVIOR_HPP_
#define BEHAVIOR_HPP_

// Distance (meters) within which an obstacle ahead triggers SLOW
constexpr double far_threshold = 10.0;

// Behavioral state: the high-level driving mode. CRUISE and SLOW are chosen by
// distance (see decide_state). STOP is NOT distance-based: it is decided by
// feasibility -- the caller enters STOP only when the local planner finds NO
// collision-free trajectory (the road is actually blocked).
//
// Design note: an early version used distance for STOP too, but that deadlocked --
// stopping near an AVOIDABLE obstacle keeps the obstacle in range, so the car stays
// stopped forever. Since the Frenet planner already swerves around avoidable
// obstacles, STOP should mean "no way through", not "something is nearby".
enum class State
{
    CRUISE,  // clear ahead: drive at full target speed
    SLOW,    // obstacle ahead within far_threshold, but a path still exists: reduce speed
    STOP,    // no collision-free trajectory: road blocked, come to a stop
};

// Chooses the behavior state from `dist_to_obstacle`, the distance in meters to the
// nearest obstacle ahead. Returns SLOW if closer than far_threshold, otherwise CRUISE.
// Never returns STOP -- STOP is a feasibility decision made by the caller (see above).
State decide_state(double dist_to_obstacle);

#endif // BEHAVIOR_HPP_