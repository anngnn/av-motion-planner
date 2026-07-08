#include "planning/frenet.hpp"
#include "common/math_utils.hpp"

RefLine path_to_ref_line(const Path & path)
{
    RefLine refline;
    double running_s = 0;
    double heading = 0.0;
    if (path.size() < 2) { return refline; }
    for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
    {
        const Point& cur  = path.at(i);
        const Point& next = path.at(i + 1);
        heading = std::atan2(next.y - cur.y, next.x - cur.x);
        refline.push_back(RefPoint{Pose{cur.x, cur.y, heading}, running_s});
        running_s += eucl_dist(cur, next);
    }
    refline.push_back(RefPoint{Pose{path.back().x, path.back().y, heading}, running_s});
    return refline;
}

// Convert a world-frame point into Frenet (s, d) coordinates relative to the
// reference line. Two steps: 
// (1) find the nearest reference point, 
// (2) project the offset vector onto the road's along-track and cross-track directions.
FrenetPoint to_frenet(const Point & world_p, const RefLine & rline)
{
    // Step 1: min-scan for the reference point closest to world_p
    auto closest_rpoint = rline.at(0);
    auto shortest_d_to_rpoint = eucl_dist(world_p, Point{closest_rpoint.world_pos.x, closest_rpoint.world_pos.y});

    for (const auto & rpoint : rline)
    {
        auto dis = eucl_dist(world_p, Point{rpoint.world_pos.x, rpoint.world_pos.y});
        if (dis < shortest_d_to_rpoint)
        {
            closest_rpoint = rpoint;
            shortest_d_to_rpoint = dis;
        }
    }

    // Step 2: project the offset onto the road frame at the nearest point
    // (dx, dy) is the vector FROM the reference point TO the car (world_p - refpoint)
    auto dx = world_p.x - closest_rpoint.world_pos.x;
    auto dy = world_p.y - closest_rpoint.world_pos.y;
    auto heading_theta = closest_rpoint.world_pos.theta;

    // Dot the offset with the road-direction unit vector (cos, sin): how far ALONG
    auto along_track_distance = dx *   std::cos(heading_theta)  + dy * std::sin(heading_theta);
    // Dot the offset with the perpendicular unit vector (-sin, cos): how far to the SIDE
    // (-sin, cos) is the road direction rotated +90 deg CCW, so +d is left of the road
    auto cross_track_distance = dx * (-std::sin(heading_theta)) + dy * std::cos(heading_theta);

    auto d = cross_track_distance;
    // s = arc length up to the nearest ref point, PLUS how far past it along the road
    auto s = closest_rpoint.s + along_track_distance;

    return FrenetPoint{s, d};
}
