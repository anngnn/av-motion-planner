#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "planning/quintic_polynomial.hpp"

// Approx against 0 needs an absolute margin: its default relative tolerance
// collapses to zero when the expected value is 0, so tiny floating-point residue
// (e.g. -2e-16 instead of exact 0) would fail the comparison.
namespace 
{
    Catch::Approx zero() { return Catch::Approx(0.0).margin(1e-9); }
}

TEST_CASE("QuinticPolynomial meets its boundary conditions", "[frenet] [quintic]")
{
    SECTION("lane change: d 0 -> 2 over T=5, settled at both ends")
    {
        // start: at d=0, no lateral motion; end: at d=2, settled (vel/accel 0)
        double d_start = 0, d_dot_start = 0, d_ddot_start = 0;
        double d_end   = 2, d_dot_end   = 0, d_ddot_end   = 0;
        double T = 5;
        QuinticPolynomial q(d_start, d_dot_start, d_ddot_start,
                            d_end,   d_dot_end,   d_ddot_end, T);

        // start conditions at t=0 (all zero here, so use zero-margin approx)
        REQUIRE(q.calc_pos(0)  == zero());
        REQUIRE(q.calc_vel(0)  == zero());
        REQUIRE(q.calc_acc(0)  == zero());

        // end conditions at t=T
        REQUIRE(q.calc_pos(T)  == Catch::Approx(d_end));  // nonzero: 2
        REQUIRE(q.calc_vel(T)  == zero());
        REQUIRE(q.calc_acc(T)  == zero());
    }

    SECTION("nonzero start motion is respected")
    {
        // start already moving/accelerating laterally, end settled elsewhere
        double d_start = 1, d_dot_start = 0.5, d_ddot_start = 0.2;
        double d_end   = -1, d_dot_end  = 0,   d_ddot_end   = 0;
        double T = 4;
        QuinticPolynomial q(d_start, d_dot_start, d_ddot_start,
                            d_end,   d_dot_end,   d_ddot_end, T);

        // nonzero start conditions
        REQUIRE(q.calc_pos(0)  == Catch::Approx(d_start));
        REQUIRE(q.calc_vel(0)  == Catch::Approx(d_dot_start));
        REQUIRE(q.calc_acc(0)  == Catch::Approx(d_ddot_start));

        REQUIRE(q.calc_pos(T)  == Catch::Approx(d_end));  // nonzero: -1
        REQUIRE(q.calc_vel(T)  == zero());
        REQUIRE(q.calc_acc(T)  == zero());
    }
}

TEST_CASE("QuinticPolynomial degenerate cases", "[frenet] [quintic]")
{
    SECTION("all-zero conditions give the zero polynomial")
    {
        QuinticPolynomial q(0, 0, 0, 0, 0, 0, 5);
        // should be 0 everywhere along the horizon
        REQUIRE(q.calc_pos(0)   == zero());
        REQUIRE(q.calc_pos(2.5) == zero());
        REQUIRE(q.calc_pos(5)   == zero());
    }

    SECTION("holding a constant offset stays flat")
    {
        // start and end both at d=3, no motion: the poly should sit at 3 throughout
        QuinticPolynomial q(3, 0, 0, 3, 0, 0, 5);
        REQUIRE(q.calc_pos(0)   == Catch::Approx(3));
        REQUIRE(q.calc_pos(2.5) == Catch::Approx(3));
        REQUIRE(q.calc_pos(5)   == Catch::Approx(3));
        // and never drifts sideways
        REQUIRE(q.calc_vel(2.5) == zero());
    }
}
