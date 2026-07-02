#ifndef ROAD_GRAPH_HPP_
#define ROAD_GRAPH_HPP_
// Road network represented as a graph: nodes are waypoints/intersections,
// edges are road segments. Used by A* to find a sequence of waypoints from
// start to goal.
#include <vector>
#include <unordered_map>

#include "common/types.hpp"

// A waypoint or intersection in the road network
struct Node
{
    int id;
    Point pos;  // world-frame position in meters
};

// A directed connection from one node to another with a travel cost
struct Edge
{
    int to;       // destination node id
    double cost;  // Euclidean distance in meters
};

// Undirected road graph stored as an adjacency list.
// Nodes are looked up by integer id in O(1).
// add_edge() adds both directions so the graph is always symmetric.
class RoadGraph
{
    public:
        // Register a waypoint; must be called before any edge references this id
        void add_node(int id, const Point& pos);

        // Connect two nodes; cost is computed as Euclidean distance between them
        void add_edge(int from, int to);

        // Return the node with this id (throws if id unknown)
        const Node& node(int id) const { return nodes_.at(id); }

        // Return all edges leaving this node (throws if id unknown)
        const std::vector<Edge>& neighbors(int id) const { return adj_.at(id); }

    private:
        std::unordered_map<int, Node> nodes_;              // id → node
        std::unordered_map<int, std::vector<Edge>> adj_;   // id → outgoing edges
};

#endif // ROAD_GRAPH_HPP_