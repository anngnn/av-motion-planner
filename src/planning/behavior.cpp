#include "planning/behavior.hpp"

// Only CRUISE vs SLOW here -- STOP is a feasibility decision (no safe trajectory)
// made by the caller, not a distance threshold. See behavior.hpp for why.
State decide_state(double dist_to_obstacle)
{
    if (dist_to_obstacle < far_threshold)
    {
        return State::SLOW;
    }
    return State::CRUISE;
}
