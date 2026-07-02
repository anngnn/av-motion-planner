#ifndef A_STAR_HPP_
#define A_STAR_HPP_

#include <optional>
#include "common/types.hpp"
#include "planning/road_graph.hpp"

std::optional<Path> a_star(const RoadGraph & rg, int start, int goal);

#endif // A_STAR_HPP_