#ifndef QUARTIC_POLYNOMIAL_HPP_
#define QUARTIC_POLYNOMIAL_HPP_

// Quartic (4th order) polynomial for longitudinal motion s(t) over a maneuver.
// Unlike the quintic, the END POSITION is left free: fix end velocity and
// acceleration (reach cruising speed smoothly) but not where along the road to
// end up. That is one fewer constraint, so degree 4 (5 coefficients) not 5.
class QuarticPolynomial
{
    public:
        // Boundary conditions: start pos/vel/accel, but only end vel/accel (no end pos).
        // T is the maneuver horizon; the coefficients are solved once here.
        QuarticPolynomial(double s_start, double s_dot_start, double s_ddot_start,
                          double s_dot_end, double s_ddot_end,
                          double T);

        // Evaluate the polynomial (and its derivatives) at a query time t in [0, T].
        double calc_pos(double t)  const;  // position s(t)
        double calc_vel(double t)  const;  // 1st derivative: longitudinal velocity
        double calc_acc(double t)  const;  // 2nd derivative: longitudinal acceleration
        double calc_jerk(double t) const;  // 3rd derivative: longitudinal jerk

    private:
        double a0_, a1_, a2_, a3_, a4_;  // solved coefficients of s(t)
};

#endif // QUARTIC_POLYNOMIAL_HPP_