#include "planning/behavior.hpp"

State decide_state(double dist_to_obstacle)
{
    if (dist_to_obstacle < stop_threshold)
    {
        return State::STOP;
    } 
    else if (dist_to_obstacle < far_threshold)
    {
        return State::SLOW;
    } else
    {
        return State::CRUISE;
    }
    
}
