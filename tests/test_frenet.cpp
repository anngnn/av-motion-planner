#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "planning/frenet.hpp"
#include "common/math_utils.hpp"  // kPi

TEST_CASE("path_to_ref_line on a straight line", "[frenet] [path_to_ref_line]")
{
    SECTION("3 points along +x axis: s accumulates, heading stays 0")
    {
        Path p{ Point{0, 0}, Point{2, 0}, Point{4, 0} };
        RefLine rline = path_to_ref_line(p);

        REQUIRE(rline.size() == 3);

        // positions carry through unchanged
        REQUIRE(rline.at(0).world_pos.x == 0);
        REQUIRE(rline.at(1).world_pos.x == 2);
        REQUIRE(rline.at(2).world_pos.x == 4);

        // s is cumulative arc length from the start
        REQUIRE(rline.at(0).s == Catch::Approx(0));
        REQUIRE(rline.at(1).s == Catch::Approx(2));
        REQUIRE(rline.at(2).s == Catch::Approx(4));

        // all segments point along +x, so heading is 0 everywhere
        REQUIRE(rline.at(0).world_pos.theta == Catch::Approx(0));
        REQUIRE(rline.at(1).world_pos.theta == Catch::Approx(0));
        REQUIRE(rline.at(2).world_pos.theta == Catch::Approx(0));
    }
}

TEST_CASE("path_to_ref_line heading follows the path direction", "[frenet] [path_to_ref_line]")
{
    SECTION("right turn then up: heading goes from 0 to pi/2")
    {
        // 0->1 travels +x (heading 0), 1->2 travels +y (heading pi/2)
        Path p{ Point{0, 0}, Point{3, 0}, Point{3, 4} };
        RefLine rline = path_to_ref_line(p);

        REQUIRE(rline.size() == 3);

        // s: 0, then 3 (horizontal leg), then 3 + 4 = 7 (vertical leg)
        REQUIRE(rline.at(0).s == Catch::Approx(0));
        REQUIRE(rline.at(1).s == Catch::Approx(3));
        REQUIRE(rline.at(2).s == Catch::Approx(7));

        // headings: first leg +x, second leg +y; last point reuses previous heading
        REQUIRE(rline.at(0).world_pos.theta == Catch::Approx(0));
        REQUIRE(rline.at(1).world_pos.theta == Catch::Approx(kPi / 2));
        REQUIRE(rline.at(2).world_pos.theta == Catch::Approx(kPi / 2));
    }
}

TEST_CASE("path_to_ref_line handles degenerate paths", "[frenet] [path_to_ref_line]")
{
    SECTION("empty path returns empty refline")
    {
        Path p{};
        REQUIRE(path_to_ref_line(p).empty());
    }

    SECTION("single point returns empty refline")
    {
        Path p{ Point{1, 1} };
        REQUIRE(path_to_ref_line(p).empty());
    }

    SECTION("diagonal segment gives 45-degree heading and correct s")
    {
        // 3-4-5 triangle: distance is 5, heading atan2(4,3)
        Path p{ Point{0, 0}, Point{3, 4} };
        RefLine rline = path_to_ref_line(p);

        REQUIRE(rline.size() == 2);
        REQUIRE(rline.at(1).s == Catch::Approx(5));
        REQUIRE(rline.at(0).world_pos.theta == Catch::Approx(std::atan2(4, 3)));
    }
}


TEST_CASE("to_frenet", "[frenet] [to_frenet]")
{
    Path p{ Point{0, 0}, Point{10, 0} };
    RefLine rline = path_to_ref_line(p);

    SECTION("on the line")
    {
        FrenetPoint fpoint = to_frenet(Point {4, 0}, rline);
        REQUIRE(fpoint.s == Catch::Approx(4));
        REQUIRE(fpoint.d == Catch::Approx(0));
    }
    SECTION("left of the line")
    {
        // +y is left of a road heading +x, so d is positive
        FrenetPoint fpoint = to_frenet(Point {4, 2}, rline);
        REQUIRE(fpoint.s == Catch::Approx(4));
        REQUIRE(fpoint.d == Catch::Approx(2));
    }
    SECTION("right of the line")
    {
        // -y is right of the road, so d is negative
        FrenetPoint fpoint = to_frenet(Point {4, -3}, rline);
        REQUIRE(fpoint.s == Catch::Approx(4));
        REQUIRE(fpoint.d == Catch::Approx(-3));
    }
}

TEST_CASE("from_frenet", "[frenet] [from_frenet]")
{
    // reference line along +x: s == x, heading 0, so +d is +y (left)
    Path p{ Point{0, 0}, Point{10, 0} };
    RefLine rline = path_to_ref_line(p);

    SECTION("on the line: d=0 maps back to the reference point")
    {
        Point w = from_frenet(FrenetPoint{4, 0}, rline);
        REQUIRE(w.x == Catch::Approx(4));
        REQUIRE(w.y == Catch::Approx(0).margin(1e-9));
    }
    SECTION("left offset: +d lands at +y")
    {
        Point w = from_frenet(FrenetPoint{4, 2}, rline);
        REQUIRE(w.x == Catch::Approx(4));
        REQUIRE(w.y == Catch::Approx(2));
    }
    SECTION("right offset: -d lands at -y")
    {
        Point w = from_frenet(FrenetPoint{4, -3}, rline);
        REQUIRE(w.x == Catch::Approx(4));
        REQUIRE(w.y == Catch::Approx(-3));
    }
}

TEST_CASE("to_frenet and from_frenet round-trip", "[frenet] [from_frenet]")
{
    Path p{ Point{0, 0}, Point{10, 0} };
    RefLine rline = path_to_ref_line(p);

    SECTION("world -> frenet -> world returns the original point")
    {
        Point original{6, 1.5};
        FrenetPoint f = to_frenet(original, rline);
        Point back = from_frenet(f, rline);
        REQUIRE(back.x == Catch::Approx(original.x));
        REQUIRE(back.y == Catch::Approx(original.y));
    }
}