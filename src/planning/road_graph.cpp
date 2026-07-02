// Implementation of RoadGraph: node/edge insertion and adjacency queries.
#include "planning/road_graph.hpp"

void RoadGraph::add_node(int id, const Point & pos)
{
    nodes_[id] = Node{id, pos};
    adj_[id] = {};  //  initialize an empty entry in adj_
}

void RoadGraph::add_edge(int from, int to)
{
    double edge_cost = eucl_dist(nodes_.at(from).pos, nodes_.at(to).pos);
    adj_[from].push_back(Edge{to, edge_cost});
    adj_[to].push_back(Edge{from, edge_cost});
}
