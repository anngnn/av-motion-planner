// A* shortest-path search on a RoadGraph using Euclidean distance heuristic.
#include "planning/astar.hpp"
#include "common/math_utils.hpp"

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

struct OpenEntry
{
    int id;
    double f;
};

Path reconstruct_path(const RoadGraph & rg, const std::unordered_map<int, int> & parent, int id)
{
    Path path;
    int cur = id;
    while (cur != -1)
    {
        auto p = rg.node(cur).pos;
        path.push_back(p);
        cur = parent.at(cur);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::optional<Path> a_star(const RoadGraph & rg, int start_id, int goal_id)
{
    auto cmp = [](const OpenEntry & a, const OpenEntry & b){ return a.f > b.f; };
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, decltype(cmp)> pq(cmp);
    std::unordered_set<int> visited;
    std::unordered_map<int, double> g_score; // best known cost from start to each node
    std::unordered_map<int, int> parent;

    pq.push(OpenEntry{start_id, 0.0 + eucl_dist(rg.node(start_id).pos, rg.node(goal_id).pos)});
    g_score[start_id] = 0.0;
    parent[start_id] = -1;

    while (!pq.empty())
    {
        auto cur = pq.top();
        pq.pop();

        if (visited.contains(cur.id)) continue;
        visited.insert(cur.id);

        if (cur.id == goal_id)
        {
            return reconstruct_path(rg, parent, cur.id);
        }

        auto neighbors = rg.neighbors(cur.id);
        for (const auto & n : neighbors)
        {
            // check visited
            if (visited.contains(n.to)) continue;
            auto g = g_score[cur.id] + n.cost;
            auto f = g + eucl_dist(rg.node(n.to).pos, rg.node(goal_id).pos);
            // add to pq
            pq.push(OpenEntry(n.to, f));
            // add to gscore
            g_score[n.to] = g;
            // update parent
            parent[n.to] = cur.id;
        }
    }

    return std::nullopt;
}
