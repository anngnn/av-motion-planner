#include "common/math_utils.hpp"
#include "planning/frenet.hpp"
#include "planning/quintic_polynomial.hpp"
#include "planning/quartic_polynomial.hpp"

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
    auto min_dis_to_rpoint = eucl_dist(world_p, Point{closest_rpoint.world_pos.x, closest_rpoint.world_pos.y});

    for (const auto & rpoint : rline)
    {
        auto dis = eucl_dist(world_p, Point{rpoint.world_pos.x, rpoint.world_pos.y});
        if (dis < min_dis_to_rpoint)
        {
            closest_rpoint = rpoint;
            min_dis_to_rpoint = dis;
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

// Convert a Frenet (s, d) coordinate back into a world-frame point, the inverse of
// to_frenet. Three steps:
// (1) find the two ref points that bracket the target arc length s,
// (2) linearly interpolate base position and heading between them,
// (3) step d meters perpendicular to that heading.
Point from_frenet(const FrenetPoint & fp, const RefLine & rline)
{
    auto s = fp.s;
    auto d = fp.d;

    // Step 1: find the bracketing pair where lower.s <= s <= upper.s
    auto rpoint_lower = rline.at(0);
    auto rpoint_upper = rline.at(0);
    for (int i = 0; i < static_cast<int>(rline.size()) - 1; ++i)
    {
        if (rline.at(i + 1).s >= s)
        {
            rpoint_lower = rline.at(i);
            rpoint_upper = rline.at(i + 1);
            break;
        }
    }

    auto ratio = (s - rpoint_lower.s) / (rpoint_upper.s - rpoint_lower.s);

    auto base_x = rpoint_lower.world_pos.x     + ratio*(rpoint_upper.world_pos.x - rpoint_lower.world_pos.x);
    auto base_y = rpoint_lower.world_pos.y     + ratio*(rpoint_upper.world_pos.y - rpoint_lower.world_pos.y);
    auto theta  = rpoint_lower.world_pos.theta + ratio*(rpoint_upper.world_pos.theta - rpoint_lower.world_pos.theta);
   
    auto x = base_x + d * std::cos(theta + kPi / 2);
    auto y = base_y + d * std::sin(theta + kPi / 2);

    return Point{x, y};
}

std::vector<FrenetTrajectory> generate_frenet_trajectories(
    const FrenetState & start, const RefLine & rline, const FrenetConfig & config)
{   
    std::vector<FrenetTrajectory> trajs;
    double d_step = (2 * config.max_road_width) / (config.num_d_samples - 1);
    for (int i = 0; i < config.num_d_samples; ++i)
    {
        double d_end = -config.max_road_width + i * d_step;
        double t_step = (config.max_t - config.min_t) / (config.num_t_samples - 1);
        for (int j = 0; j < config.num_t_samples; ++j)
        {
            double T = config.min_t + j * t_step;
            double speed_step = (2 * config.speed_range) / (config.num_speed_samples - 1);
            for (int k = 0; k < config.num_speed_samples; ++k)
            {
                double speed = (config.target_speed - config.speed_range) + k * speed_step;
                QuinticPolynomial lat(start.d, start.d_dot, start.d_ddot,
                                    d_end, 0.0, 0.0,
                                    T);
                QuarticPolynomial lon(start.s, start.s_dot, start.s_ddot,
                                    speed, 0.0,
                                    T);
                FrenetTrajectory traj;

                for (double t = 0.0; t <= T; t += config.dt)
                {
                    double s = lon.calc_pos(t);
                    double d = lat.calc_pos(t);
                    traj.t.push_back(t);

                    // sample the longitudinal (forward) curve and its derivatives
                    traj.s.push_back(s);
                    traj.s_dot.push_back(lon.calc_vel(t));
                    traj.s_ddot.push_back(lon.calc_acc(t));
                    traj.s_dddot.push_back(lon.calc_jerk(t));

                    // sample the lateral (sideways) curve and its derivatives
                    traj.d.push_back(d);
                    traj.d_dot.push_back(lat.calc_vel(t));
                    traj.d_ddot.push_back(lat.calc_acc(t));
                    traj.d_dddot.push_back(lat.calc_jerk(t));

                    Point point_world = from_frenet(FrenetPoint{s, d}, rline);
                    traj.x_world.push_back(point_world.x);
                    traj.y_world.push_back(point_world.y);
                }
                trajs.push_back(traj);
            }
        }
    }
    return trajs;
}