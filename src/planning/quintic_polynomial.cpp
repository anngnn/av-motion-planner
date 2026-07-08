// Implements QuinticPolynomial: solves for the 6 coefficients of d(t) given the
// start and end boundary conditions, then evaluates the polynomial and derivatives.
#include <Eigen/Dense>
#include "planning/quintic_polynomial.hpp"

QuinticPolynomial::QuinticPolynomial(double d_start, double d_dot_start, double d_ddot_start,
                                     double d_end,   double d_dot_end,   double d_ddot_end, double T)
    // a0, a1, a2 fall straight out of the start conditions at t=0:
    // pos(0)=a0, vel(0)=a1, acc(0)=2*a2
    : a0_(d_start), a1_(d_dot_start), a2_(d_ddot_start / 2.0)
{
    // a3, a4, a5 come from the three end conditions at t=T, giving A * [a3 a4 a5]^T = b
    const double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

    // rows are pos, vel, acc of the a3..a5 terms evaluated at T
    Eigen::Matrix3d A;
    A << T3,    T4,     T5,
         3*T2,  4*T3,   5*T4,
         6*T,   12*T2,  20*T3;

    // b = desired end (pos, vel, acc) minus the part already fixed by a0, a1, a2
    Eigen::Vector3d b;
    b << d_end       - (a0_ + a1_*T + a2_*T2),
         d_dot_end   - (a1_ + 2*a2_*T),
         d_ddot_end  - 2*a2_;

    // QR solve is more numerically stable than inverting A
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    a3_ = x(0);  a4_ = x(1);  a5_ = x(2);
}

double QuinticPolynomial::calc_pos(double t) const
{
    const double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
    return a0_ + a1_*t + a2_*t2 + a3_*t3 + a4_*t4 + a5_*t5;
}
double QuinticPolynomial::calc_vel(double t) const
{
    const double t2 = t*t, t3 = t2*t, t4 = t3*t;
    return a1_ + 2*a2_*t + 3*a3_*t2 + 4*a4_*t3 + 5*a5_*t4;
}
double QuinticPolynomial::calc_acc(double t) const
{
    const double t2 = t*t, t3 = t2*t;
    return 2*a2_ + 6*a3_*t + 12*a4_*t2 + 20*a5_*t3;
}
double QuinticPolynomial::calc_jerk(double t) const
{
    const double t2 = t*t;
    return 6*a3_ + 24*a4_*t + 60*a5_*t2;
}