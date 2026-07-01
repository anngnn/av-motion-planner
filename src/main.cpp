#include <iostream>
#include "vehicle/kinematic_model.hpp"

int main()
{
    // start at origin, facing +x (theta = 0)
    KinematicModel car{Pose{0.0, 0.0, 0.0}};

    // drive straight for 10 steps: full throttle, no steering
    for (int i = 0; i < 10; ++i)
    {
        car.update(1.0, 0.0, 0.1);
        std::cout   << "x: " << car.pose().x
                    << " y: " << car.pose().y
                    << " theta: " << car.pose().theta
                    << " speed: " << car.speed() << "\n";
    }
    return 0;
}