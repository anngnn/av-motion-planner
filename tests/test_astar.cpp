#include <catch2/catch_test_macros.hpp>
#include "planning/astar.hpp"

TEST_CASE("test chooseing right path", "[astar]")
{
    SECTION("3 nodes, 2 paths, different costs")
    {
        RoadGraph rg;
        rg.add_node(0, Point{0.0, 0.0});
        rg.add_node(1, Point{0.0, 5.0});
        rg.add_node(2, Point{10.0, 0.0});
        rg.add_edge(0, 1);
        rg.add_edge(1, 2);
        rg.add_edge(0, 2);
        auto path = a_star(rg, 0, 2);
        REQUIRE(path.has_value());
        REQUIRE((*path).size() == 2);
        REQUIRE((*path).back().x == 10.0);
        REQUIRE((*path).back().y == 0.0);
    }
}

TEST_CASE("no path exists", "[astar]")
{
    SECTION("disconnected graph returns nullopt")
    {
        RoadGraph rg;
        rg.add_node(0, Point{0.0, 0.0});
        rg.add_node(1, Point{10.0, 0.0});
        // no edge added — nodes are isolated
        auto path = a_star(rg, 0, 1);
        REQUIRE(!path.has_value());
    }
}

TEST_CASE("start equals goal", "[astar]")
{
    SECTION("path contains only the start node")
    {
        RoadGraph rg;
        rg.add_node(0, Point{3.0, 4.0});
        auto path = a_star(rg, 0, 0);
        REQUIRE(path.has_value());
        REQUIRE((*path).size() == 1);
        REQUIRE((*path).at(0).x == 3.0);
        REQUIRE((*path).at(0).y == 4.0);
    }
}

TEST_CASE("multi-hop path", "[astar]")
{
    SECTION("4 nodes in a line, correct intermediate nodes returned")
    {
        RoadGraph rg;
        rg.add_node(0, Point{0.0, 0.0});
        rg.add_node(1, Point{1.0, 0.0});
        rg.add_node(2, Point{2.0, 0.0});
        rg.add_node(3, Point{3.0, 0.0});
        rg.add_edge(0, 1);
        rg.add_edge(1, 2);
        rg.add_edge(2, 3);
        auto path = a_star(rg, 0, 3);
        REQUIRE(path.has_value());
        REQUIRE((*path).size() == 4);
        REQUIRE((*path).at(1).x == 1.0);  // passes through node 1
        REQUIRE((*path).at(2).x == 2.0);  // passes through node 2
        REQUIRE((*path).back().x == 3.0); // ends at goal
    }
}