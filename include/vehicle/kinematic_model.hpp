#ifndef VEHICLE_KINEMATIC_MODEL_HPP_
#define VEHICLE_KINEMATIC_MODEL_HPP_

#include "common/types.hpp"
#include <cmath>

// Physical constants for a typical sedan (e.g. Toyota Camry)
// constexpr = known at compile time, zero runtime cost
constexpr double kWheelbase   = 2.7;  // distance between front and rear axles, meters
constexpr double kMaxSteering = 0.6;  // maximum steering angle, radians (~34 degrees)

class KinematicModel
{
public:
    // Takes the starting pose (where the car is and which way it faces)
    // explicit prevents accidental implicit conversion from a Pose to a KinematicModel
    explicit KinematicModel(Pose initial_pose)
        : pose_(initial_pose), speed_(0.0), steering_(0.0) {}

    // Advances the car's state by one timestep using the bicycle model equations
    // acceleration:    change in speed this step, m/s^2 (positive = gas, negative = brake)
    // steering_angle:  front wheel angle in radians, clamped to [-kMaxSteering, kMaxSteering]
    // dt:              timestep in seconds
    void update(double acceleration, double steering_angle, double dt);

    // Getter returns current pose (position + heading)
    const Pose & pose() const { return pose_; }

    // Primitive — return by value, copying a double is free
    double speed() const { return speed_; }

private:
    Pose   pose_;      // current position and heading
    double speed_;     // current speed in m/s
    double steering_;  // current steering angle in radians (kept for visualization)
};

#endif  // VEHICLE_KINEMATIC_MODEL_HPP_
