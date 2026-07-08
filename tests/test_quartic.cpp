#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "planning/quartic_polynomial.hpp"

// Approx against 0 needs an absolute margin: its default relative tolerance
// collapses to zero when the expected value is 0, so tiny floating-point residue
// (e.g. -2e-16 instead of exact 0) would fail the comparison.
namespace
{
    Catch::Approx zero() { return Catch::Approx(0.0).margin(1e-9); }
}

TEST_CASE("QuarticPolynomial meets its boundary conditions", "[frenet] [quartic]")
{
    SECTION("accelerate 5 -> 10 m/s over T=4, end position is NOT constrained")
    {
        // cruising: start 5 m/s, speed up to 10 m/s, no accel at either end
        double s_start = 0, s_dot_start = 5, s_ddot_start = 0;
        double s_dot_end = 10, s_ddot_end = 0;
        double T = 4;
        QuarticPolynomial q(s_start, s_dot_start, s_ddot_start,
                            s_dot_end, s_ddot_end, T);

        // start conditions at t=0
        REQUIRE(q.calc_pos(0) == zero());               // s_start = 0
        REQUIRE(q.calc_vel(0) == Catch::Approx(5));
        REQUIRE(q.calc_acc(0) == zero());

        // end conditions at t=T: only velocity and accel (position is free)
        REQUIRE(q.calc_vel(T) == Catch::Approx(10));
        REQUIRE(q.calc_acc(T) == zero());
    }

    SECTION("nonzero start accel is respected")
    {
        double s_start = 2, s_dot_start = 3, s_ddot_start = 1;
        double s_dot_end = 6, s_ddot_end = 0;
        double T = 5;
        QuarticPolynomial q(s_start, s_dot_start, s_ddot_start,
                            s_dot_end, s_ddot_end, T);

        REQUIRE(q.calc_pos(0) == Catch::Approx(2));
        REQUIRE(q.calc_vel(0) == Catch::Approx(3));
        REQUIRE(q.calc_acc(0) == Catch::Approx(1));

        REQUIRE(q.calc_vel(T) == Catch::Approx(6));
        REQUIRE(q.calc_acc(T) == zero());
    }
}

TEST_CASE("QuarticPolynomial constant-speed cruise stays linear", "[frenet] [quartic]")
{
    SECTION("same start/end speed, no accel: position advances linearly")
    {
        // 4 m/s throughout, no acceleration anywhere -> s(t) = s0 + v*t
        double s_start = 0, s_dot_start = 4, s_ddot_start = 0;
        double s_dot_end = 4, s_ddot_end = 0;
        double T = 3;
        QuarticPolynomial q(s_start, s_dot_start, s_ddot_start,
                            s_dot_end, s_ddot_end, T);

        // constant velocity means position is exactly v*t
        REQUIRE(q.calc_pos(0) == zero());
        REQUIRE(q.calc_pos(1) == Catch::Approx(4));
        REQUIRE(q.calc_pos(2) == Catch::Approx(8));
        REQUIRE(q.calc_pos(3) == Catch::Approx(12));

        // velocity holds at 4, accel stays 0 the whole way
        REQUIRE(q.calc_vel(1.5) == Catch::Approx(4));
        REQUIRE(q.calc_acc(1.5) == zero());
    }
}
