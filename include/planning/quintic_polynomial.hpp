#ifndef QUINTIC_POLYNOMIAL_HPP_
#define QUINTIC_POLYNOMIAL_HPP_

// Quintic (5th order) polynomial for lateral motion d(t) over a maneuver.
// Fully constrained: start and end each fix position, velocity, and acceleration.
class QuinticPolynomial
{
    public:
        // Boundary conditions: lateral pos/vel/accel at the start (t=0) and end (t=T).
        // T is the maneuver horizon (total duration); the coefficients are solved once here.
        QuinticPolynomial(double d_start, double d_dot_start, double d_ddot_start,
                          double d_end,   double d_dot_end,   double d_ddot_end,
                          double T);

        // Evaluate the polynomial (and its derivatives) at a query time t in [0, T].
        // T is the fixed horizon; t is any moment within it to sample the state at.
        double calc_pos(double t)  const;  // position d(t)
        double calc_vel(double t)  const;  // 1st derivative: lateral velocity
        double calc_acc(double t)  const;  // 2nd derivative: lateral acceleration
        double calc_jerk(double t) const;  // 3rd derivative: lateral jerk

    private:
        double a0_, a1_, a2_, a3_, a4_, a5_;  // solved coefficients of d(t)
};

#endif // QUINTIC_POLYNOMIAL_HPP_