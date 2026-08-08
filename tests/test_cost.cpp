#include <catch2/catch_test_macros.hpp>
#include "planning/frenet.hpp"

// Helpers to build minimal FrenetTrajectory objects with only the fields a given
// test needs. compute_cost reads s_dddot/d_dddot (jerk), d.back(), s_dot.back(),
// and x_world/y_world (obstacle proximity); is_collision reads only x_world/y_world.
namespace
{
// A trajectory with lateral offsets, speeds, and jerks set (no world points).
FrenetTrajectory frenet_traj(std::vector<double> d,
                             std::vector<double> s_dot,
                             std::vector<double> s_dddot = {},
                             std::vector<double> d_dddot = {})
{
    FrenetTrajectory t;
    t.d = std::move(d);
    t.s_dot = std::move(s_dot);
    t.s_dddot = std::move(s_dddot);
    t.d_dddot = std::move(d_dddot);
    return t;
}

// A trajectory defined by its world-frame points (for collision / proximity tests).
FrenetTrajectory world_traj(std::vector<double> xs, std::vector<double> ys)
{
    FrenetTrajectory t;
    t.x_world = std::move(xs);
    t.y_world = std::move(ys);
    return t;
}
}  // namespace

// ---------------------------------------------------------------------------
// compute_cost: test each penalty term in isolation by zeroing the other weights,
// and check relative ordering (the better trajectory scores lower).
// ---------------------------------------------------------------------------

TEST_CASE("compute_cost off-center penalty", "[frenet] [cost]")
{
    FrenetConfig config;                 // target_speed = 5 by default
    CostWeights w{0.0, 1.0, 0.0, 0.0};   // only w_offcenter active
    std::vector<Obstacle> none{};

    // both at target speed, no jerk; differ only in final lateral offset
    FrenetTrajectory centered = frenet_traj({0.0}, {5.0});
    FrenetTrajectory offset   = frenet_traj({2.0}, {5.0});

    REQUIRE(compute_cost(centered, w, config, none) < compute_cost(offset, w, config, none));
}

TEST_CASE("compute_cost speed-error penalty", "[frenet] [cost]")
{
    FrenetConfig config;                 // target_speed = 5
    CostWeights w{0.0, 0.0, 1.0, 0.0};   // only w_speed_error active
    std::vector<Obstacle> none{};

    FrenetTrajectory on_target = frenet_traj({0.0}, {5.0});   // ends at target speed
    FrenetTrajectory too_slow  = frenet_traj({0.0}, {2.0});   // ends well below

    REQUIRE(compute_cost(on_target, w, config, none) < compute_cost(too_slow, w, config, none));
}

TEST_CASE("compute_cost jerk penalty", "[frenet] [cost]")
{
    FrenetConfig config;
    CostWeights w{1.0, 0.0, 0.0, 0.0};   // only w_jerk active
    std::vector<Obstacle> none{};

    FrenetTrajectory smooth = frenet_traj({0.0}, {5.0}, {0.0, 0.0}, {0.0, 0.0});
    FrenetTrajectory jerky  = frenet_traj({0.0}, {5.0}, {3.0, 3.0}, {0.0, 0.0});

    REQUIRE(compute_cost(smooth, w, config, none) < compute_cost(jerky, w, config, none));
}

TEST_CASE("compute_cost obstacle-proximity penalty", "[frenet] [cost]")
{
    FrenetConfig config;
    CostWeights w{0.0, 0.0, 0.0, 1.0};   // only w_obstacle active
    std::vector<Obstacle> obs{ Obstacle{Point{5.0, 5.0}, 0.5} };

    // same d/speed; differ only in how close their world points pass the obstacle
    FrenetTrajectory far  = world_traj({0.0}, {0.0});   // ~7 m from the obstacle
    far.d = {0.0};  far.s_dot = {5.0};
    FrenetTrajectory near = world_traj({5.0}, {4.0});   // 1 m from the obstacle
    near.d = {0.0}; near.s_dot = {5.0};

    REQUIRE(compute_cost(far, w, config, obs) < compute_cost(near, w, config, obs));
}

// ---------------------------------------------------------------------------
// is_collision: skips the first kSkipStartPoints (3) points, then flags any
// trajectory point within (obstacle.radius + car_radius) of an obstacle.
// ---------------------------------------------------------------------------

TEST_CASE("is_collision", "[frenet] [collision]")
{
    // straight line of 6 points along +x; first 3 are skipped by is_collision
    FrenetTrajectory traj = world_traj({0, 1, 2, 3, 4, 5}, {0, 0, 0, 0, 0, 0});

    SECTION("no obstacles -> no collision")
    {
        REQUIRE_FALSE(is_collision({}, traj, 1.0));
    }

    SECTION("obstacle far from the path -> no collision")
    {
        std::vector<Obstacle> obs{ Obstacle{Point{10, 10}, 1.0} };
        REQUIRE_FALSE(is_collision(obs, traj, 1.0));
    }

    SECTION("obstacle on a checked point -> collision")
    {
        // at point (4,0): dist 0 < radius 0.5 + car 1.0 = 1.5 inflated
        std::vector<Obstacle> obs{ Obstacle{Point{4, 0}, 0.5} };
        REQUIRE(is_collision(obs, traj, 1.0));
    }

    SECTION("obstacle only near skipped start points -> no collision")
    {
        // inflated reach 1.5 from (0,0) only covers points 0 and 1, both skipped;
        // point 3 (3,0) is 3 m away -> clear
        std::vector<Obstacle> obs{ Obstacle{Point{0, 0}, 0.5} };
        REQUIRE_FALSE(is_collision(obs, traj, 1.0));
    }
}
