#ifndef BEHAVIOR_HPP_
#define BEHAVIOR_HPP_

// Distance thresholds (meters) that split the range into three behavior bands
constexpr double stop_threshold = 4.0;   // closer than this -> STOP
constexpr double far_threshold  = 10.0;  // closer than this (but past stop) -> SLOW

// Behavioral state: the high-level driving mode, decided each frame from how close
// the nearest obstacle ahead is. Each state sets the planner's target speed.
enum class State
{
    CRUISE,  // clear ahead: drive at full target speed
    SLOW,    // obstacle getting close: reduce speed
    STOP,    // obstacle very close: come to a stop
};

// Chooses the behavior state from `dist_to_obstacle`, the distance in meters to the
// nearest obstacle ahead. Returns STOP if closer than stop_threshold, SLOW if closer
// than far_threshold, otherwise CRUISE.
State decide_state(double dist_to_obstacle);

#endif // BEHAVIOR_HPP_