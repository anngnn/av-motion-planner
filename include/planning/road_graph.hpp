#ifndef ROAD_GRAPH_HPP_
#define ROAD_GRAPH_HPP_
#include <vector>
#include <unordered_map>

#include "common/types.hpp"


struct Node
{
    int id;
    Point pos;
};

struct Edge
{
    int to;
    double cost;
};

class RoadGraph
{
    public:
        void add_node(int id, const Point &  pos);
        void add_edge(int from, int to);    // bidirectional
        const Node & node(int id) const { return nodes_.at(id); }
        const std::vector<Edge> & neighbors(int id) const { return adj_.at(id); }

    private:
        std::unordered_map<int, Node> nodes_;
        std::unordered_map<int, std::vector<Edge>> adj_;

};

#endif // ROAD_GRAPH_HPP_