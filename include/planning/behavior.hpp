#ifndef BEHAVIOR_HPP_
#define BEHAVIOR_HPP_

// Behavioral state: the high-level driving mode, decided by FEASIBILITY (not distance).
// CRUISE when the local planner has a collision-free trajectory and the goal isn't
// reached; STOP when it doesn't (road blocked) or the goal is reached.
//
// Design note: an earlier version chose states by distance to the nearest obstacle
// (adding a SLOW state). That deadlocked -- slowing/stopping near an AVOIDABLE obstacle
// keeps it in range, so the car froze. Since the Frenet planner already swerves around
// avoidable obstacles, STOP must mean "no way through", not "something is nearby".
enum class State
{
    CRUISE,  // a collision-free path exists and the goal isn't reached: drive it
    STOP,    // no collision-free trajectory (blocked) or goal reached: brake and hold
};

#endif // BEHAVIOR_HPP_
