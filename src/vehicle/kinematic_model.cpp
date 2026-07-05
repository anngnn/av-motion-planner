#include "vehicle/kinematic_model.hpp"
#include <algorithm>

void KinematicModel::update(double acceleration, double steering_angle, double dt)
{
    const auto delta      = std::clamp(steering_angle, -kMaxSteering, kMaxSteering);
    const auto arc_length = speed_ * dt;             // distance travelled this step (meters)
    const auto R          = kWheelbase / std::tan(delta);  // turning radius (meters)

    // position accumulates over time — each step adds to where we already are
    // pose_.theta (car's heading) not delta (front wheel angle): cos/sin project the car's
    // travel direction onto x and y axes. The car body moves in the direction it's facing,
    // not the direction the front wheel is pointed.
    pose_.x     += speed_ * std::cos(pose_.theta) * dt;
    pose_.y     += speed_ * std::sin(pose_.theta) * dt;
    pose_.theta += arc_length / R;   // heading rotates from where it currently points

    speed_    += acceleration * dt;               // speed integrates acceleration over time
    speed_     = std::clamp(speed_, 0.0, kMaxSpeed);  // cap: no reverse, no runaway top speed
    steering_  = delta;              // store clamped angle (used for visualization)
}
