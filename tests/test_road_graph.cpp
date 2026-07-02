#include <catch2/catch_test_macros.hpp>
#include "planning/road_graph.hpp"

TEST_CASE("test add_node", "[node]")
{
    SECTION("add a node, retrieve it, check the id and position")
    {
        RoadGraph rg;
        rg.add_node(10, Point{0.0, 1.0});
        REQUIRE(rg.node(10).id == 10);
        REQUIRE(rg.node(10).pos.x == 0.0);
        REQUIRE(rg.node(10).pos.y == 1.0);
    }
}

TEST_CASE("test add_edge", "[node][edge]")
{
    SECTION("add two nodes, connect them, check both directions have an edge with the right cost")
    {
        RoadGraph rg;
        rg.add_node(10, Point{0, 0});
        rg.add_node(11, Point{3, 0});
        rg.add_node(12, Point{2.0, 3.0});
        rg.add_edge(10, 11);
        rg.add_edge(10, 12);

        REQUIRE(rg.neighbors(10).at(0).cost == 3.0);
        REQUIRE(rg.neighbors(11).at(0).cost == 3.0);
        REQUIRE(rg.neighbors(10).size() == 2);
        REQUIRE(rg.neighbors(11).size() == 1);
    }
}