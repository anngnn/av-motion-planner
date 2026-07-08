// Implements QuarticPolynomial: solves for the 5 coefficients of s(t) given the
// start conditions and end velocity/acceleration (end position is left free),
// then evaluates the polynomial and its derivatives.
#include <Eigen/Dense>
#include "planning/quartic_polynomial.hpp"

QuarticPolynomial::QuarticPolynomial(double s_start, double s_dot_start, double s_ddot_start,
                                     double s_dot_end, double s_ddot_end,
                                     double T)
    // a0, a1, a2 fall straight out of the start conditions at t=0:
    // pos(0)=a0, vel(0)=a1, acc(0)=2*a2
    : a0_(s_start), a1_(s_dot_start), a2_(s_ddot_start / 2.0)
{
    // a3, a4 come from the two end conditions (vel, acc) at t=T: A * [a3 a4]^T = b
    const double T2 = T*T, T3 = T2*T;

    // rows are vel, acc of the a3, a4 terms evaluated at T
    Eigen::Matrix2d A;
    A << 3*T2,  4*T3,
         6*T,   12*T2;

    // b = desired end (vel, acc) minus the part already fixed by a1, a2
    Eigen::Vector2d b;
    b << s_dot_end   - (a1_ + 2*a2_*T),
         s_ddot_end  - 2*a2_;

    // QR solve is more numerically stable than inverting A
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    a3_ = x(0);  a4_ = x(1);
}

double QuarticPolynomial::calc_pos(double t) const
{
    const double t2 = t*t, t3 = t2*t, t4 = t3*t;
    return a0_ + a1_*t + a2_*t2 + a3_*t3 + a4_*t4;
}
double QuarticPolynomial::calc_vel(double t) const
{
    const double t2 = t*t, t3 = t2*t;
    return a1_ + 2*a2_*t + 3*a3_*t2 + 4*a4_*t3;
}
double QuarticPolynomial::calc_acc(double t) const
{
    const double t2 = t*t;
    return 2*a2_ + 6*a3_*t + 12*a4_*t2;
}
double QuarticPolynomial::calc_jerk(double t) const
{
    return 6*a3_ + 24*a4_*t;
}
