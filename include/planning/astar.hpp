#ifndef A_STAR_HPP_
#define A_STAR_HPP_
// A* graph search: finds the shortest Path through a RoadGraph from start to
// goal node. Returns nullopt if no path exists.

#include <optional>
#include "common/types.hpp"
#include "planning/road_graph.hpp"

std::optional<Path> a_star(const RoadGraph & rg, int start_id, int goal_id);

#endif // A_STAR_HPP_