#include <catch2/catch_test_macros.hpp>
#include "planning/road_graph.hpp"

TEST_CASE("add_node stores id and position", "[node]")
{
    SECTION("retrieve node by id")
    {
        RoadGraph rg;
        rg.add_node(10, Point{0.0, 1.0});
        REQUIRE(rg.node(10).id == 10);
        REQUIRE(rg.node(10).pos.x == 0.0);
        REQUIRE(rg.node(10).pos.y == 1.0);
    }
}

TEST_CASE("node with no edges has empty neighbor list", "[node]")
{
    SECTION("neighbors() does not throw and returns empty")
    {
        RoadGraph rg;
        rg.add_node(0, Point{0.0, 0.0});
        REQUIRE(rg.neighbors(0).empty());
    }
}

TEST_CASE("add_edge is bidirectional with correct cost", "[edge]")
{
    SECTION("both directions exist, cost matches Euclidean distance")
    {
        RoadGraph rg;
        rg.add_node(10, Point{0, 0});
        rg.add_node(11, Point{3, 0});
        rg.add_node(12, Point{2.0, 3.0});
        rg.add_edge(10, 11);
        rg.add_edge(10, 12);

        // edge 10→11 and reverse: horizontal distance = 3
        REQUIRE(rg.neighbors(10).at(0).cost == 3.0);
        REQUIRE(rg.neighbors(11).at(0).cost == 3.0);

        // node 10 has two outgoing edges, node 11 has one
        REQUIRE(rg.neighbors(10).size() == 2);
        REQUIRE(rg.neighbors(11).size() == 1);
    }
}

TEST_CASE("add_edge destination id is correct", "[edge]")
{
    SECTION("edge.to points to the right node")
    {
        RoadGraph rg;
        rg.add_node(0, Point{0.0, 0.0});
        rg.add_node(5, Point{1.0, 0.0});
        rg.add_edge(0, 5);
        REQUIRE(rg.neighbors(0).at(0).to == 5);
        REQUIRE(rg.neighbors(5).at(0).to == 0);
    }
}