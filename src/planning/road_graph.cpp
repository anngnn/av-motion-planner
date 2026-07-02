#include "planning/road_graph.hpp"
#include <cmath>

namespace
{
    double eucl_dist(const Point & p1, const Point & p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return std::hypot(dx, dy);
    }   
} // namespace

void RoadGraph::add_node(int id, const Point & pos)
{
    nodes_[id] = Node{id, pos};
}

void RoadGraph::add_edge(int from, int to)
{
    double edge_cost = eucl_dist(nodes_.at(from).pos, nodes_.at(to).pos);
    adj_[from].push_back(Edge{to, edge_cost});
    adj_[to].push_back(Edge{from, edge_cost});
}
