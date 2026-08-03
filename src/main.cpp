#include <iostream>
#include <opencv2/opencv.hpp>
#include <limits>

#include "common/math_utils.hpp"
#include "vehicle/kinematic_model.hpp"
#include "planning/road_graph.hpp"
#include "planning/astar.hpp"
#include "planning/frenet.hpp"
#include "planning/behavior.hpp"

constexpr double kScale = 30.0; // pixels per meter
constexpr int kWidth = 1000;
constexpr int kHeight = 1000;
constexpr double kHeadingLineLen = 20.0;
constexpr double kReachThreshold = 1.5;
constexpr int kLookaheadIdx = 5;  // pure-pursuit: aim this many steps ahead on the plan

double world_to_pix(double coord, bool is_x)
{
    if (is_x)
    {
        return coord * kScale + kWidth / 2;

    }
    return -coord * kScale + kHeight / 2;    // origin at top-left, y increases downward
}

// Draw every graph node as a labelled black circle, plus a line for each edge,
// so the road network is visible. Positions are converted world->pixels.
void draw_road_nodes(const RoadGraph & rg, cv::Mat & canvas)
{
    for (const auto & [id, node] : rg.nodes())
    {
        auto pix_x = world_to_pix(node.pos.x, true);
        auto pix_y = world_to_pix(node.pos.y, false);
        cv::circle(canvas, cv::Point(pix_x, pix_y), 15, cv::Scalar(0, 0, 0), 2);
        cv::putText(
            canvas, std::to_string(id),
            cv::Point(pix_x - 8, pix_y + 6),  // nudge to center
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2
        );      
    }
    // draw edges
    for (const auto & [id, edges] : rg.adj())
    {
        for (const auto & e : edges)
        {
            cv::Point p1(world_to_pix(rg.node(id).pos.x, true), world_to_pix(rg.node(id).pos.y, false));
            cv::Point p2(world_to_pix(rg.node(e.to).pos.x, true), world_to_pix(rg.node(e.to).pos.y, false));
            cv::line(canvas, p1, p2, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        }
    }
}

// Draw the planned A* route as a green polyline connecting its waypoints in order.
void draw_astar_path(const Path & path, cv::Mat & canvas)
{
    // convert Path (vector<Point>) to vector<cv::Point>
    std::vector<cv::Point> pix_pts;
    for (const auto p : path)
    {
        pix_pts.push_back(cv::Point(world_to_pix(p.x, true), world_to_pix(p.y, false)));
    }
    // draw: isClosed=false, color=green, thickness=2
    cv::polylines(canvas, pix_pts, false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

// Draw the car as a filled blue circle with a red line showing its heading.
void draw_car(cv::Mat & canvas, KinematicModel & car)
{
    auto car_pix_x = world_to_pix(car.pose().x, true);
    auto car_pix_y = world_to_pix(car.pose().y, false);

    cv::circle(canvas, cv::Point(car_pix_x, car_pix_y), 8, cv::Scalar(255, 0, 0), -1);
    
    cv::Point p1(car_pix_x, car_pix_y);
    auto end_x = car_pix_x + std::cos(car.pose().theta) * kHeadingLineLen;
    auto end_y = car_pix_y - std::sin(car.pose().theta) * kHeadingLineLen;
    cv::Point p2(end_x, end_y);

    int thickness = 2;

    cv::line(canvas, p1, p2, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA);
}

// Advance the car one timestep toward the current waypoint (path[wp_id]):
// steer toward it, then if within kReachThreshold, retarget the next one.
// wp_id is taken by reference so the advance persists across frames.
void step_toward_waypoint(const Path & path, int & wp_id, KinematicModel & car)
{
    Point goal_wp = path.at(wp_id);
    // point the wheels at the target: desired heading minus current heading,
    // wrapped to [-pi, pi] so the car always turns the short way
    auto desired_heading = std::atan2(goal_wp.y - car.pose().y, goal_wp.x - car.pose().x);
    auto steering = normalize_angle(desired_heading - car.pose().theta);

    car.update(0.5, steering, 0.05);  // constant gentle acceleration, computed steering
    // close enough to this waypoint -> start heading for the next one
    if ( eucl_dist(Point{car.pose().x, car.pose().y}, goal_wp) < kReachThreshold)
    {
        wp_id++;
    }

}

// Drive the car one step along the chosen Frenet trajectory (pure pursuit).
// The trajectory is replanned every frame from the car's current pose, so
// don't track progress along it - just aim at a fixed lookahead point a few
// steps ahead on the fresh plan, steer toward it, and advance.
void follow_trajectory(const FrenetTrajectory & traj, KinematicModel & car)
{
    // fixed lookahead point on the plan, clamped so a short trajectory can't overflow
    int lookahead_idx = std::min(kLookaheadIdx, static_cast<int>(traj.x_world.size() - 1));
    Point goal_wp = Point{traj.x_world[lookahead_idx], traj.y_world[lookahead_idx]};

    // steer toward the lookahead point: heading error wrapped to the shortest turn
    auto desired_heading = std::atan2(goal_wp.y - car.pose().y, goal_wp.x - car.pose().x);
    auto steering = normalize_angle(desired_heading - car.pose().theta);

    car.update(0.5, steering, 0.05);  // constant gentle acceleration, computed steering
}

// Draw one trajectory as a polyline in the given color and thickness.
void draw_traj(const FrenetTrajectory & traj, cv::Mat & canvas, cv::Scalar color, int thickness)
{
    std::vector<cv::Point> pix_pts;
    for (size_t i = 0; i < traj.x_world.size(); ++i)
    {
        double pix_x = world_to_pix(traj.x_world[i], true);
        double pix_y = world_to_pix(traj.y_world[i], false);
        pix_pts.push_back(cv::Point(pix_x, pix_y));
    }
    cv::polylines(canvas, pix_pts, false, color, thickness, cv::LINE_AA);
}

// Draw all candidate trajectories faint gray so they sit behind the chosen one.
void draw_trajectories(const std::vector<FrenetTrajectory> & trajs, cv::Mat & canvas)
{
    for (const auto & traj : trajs)
    {
        draw_traj(traj, canvas, cv::Scalar(200, 200, 200), 1);
    }
}

// Draw each obstacle as a filled orange circle. The radius is in meters, so scale
// it to pixels the same way positions are scaled.
void draw_obstacles(const std::vector<Obstacle> & obs, cv::Mat & canvas)
{
    for (const auto & ob : obs)
    {
        auto pix_x = world_to_pix(ob.pos.x, true);
        auto pix_y = world_to_pix(ob.pos.y, false);
        int pix_radius = static_cast<int>(ob.radius * kScale);  // meters -> pixels
        cv::circle(canvas, cv::Point(pix_x, pix_y), pix_radius, cv::Scalar(0, 140, 255), -1);
    }
}

// Returns the distance in meters from `car_pos` to the nearest obstacle in `obs`.
// Returns a very large value when `obs` is empty, so the caller reads it as
// "nothing close" and stays in CRUISE.
double dist_to_nearest_obstacle(const Point & car_pos, const std::vector<Obstacle> & obs)
{
    if (obs.empty())    // no obstacles -> effectively infinite clearance
    {
        return std::numeric_limits<double>::max();
    }
    double smallest_d = eucl_dist(car_pos, obs.at(0).pos);
    for (const auto & ob : obs)
    {
        double cur_d = eucl_dist(car_pos, ob.pos);
        if (cur_d < smallest_d)
        {
            smallest_d = cur_d;
        }
    }
    return smallest_d;
}

int main()
{
    cv::Mat canvas(kHeight, kWidth, CV_8UC3);

    // Build a 3x3 grid road network. Node id = row*3 + col, so ids run 0-8
    // bottom-left to top-right. Two passes: add all nodes first, then edges
    // (add_edge looks up both endpoints, so every node must exist before any edge).
    RoadGraph rg;

    int spacing = 6;  // meters between adjacent grid nodes
    // Pass 1: place the 9 nodes. Subtracting `spacing` re-centers the grid on the
    // origin (columns land at -spacing, 0, +spacing) so it draws in the screen center.
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            int id = row * 3 + col;
            double x = col * spacing - spacing;
            double y = row * spacing - spacing;
            rg.add_node(id, Point{x, y});
        }
    }

    // Pass 2: connect each node to its right and top neighbor only. add_edge is
    // bidirectional, so this covers every grid connection exactly once.
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            int id = row * 3 + col;
            // right neighbor: same row, next column
            if (col < 2)
            {
                rg.add_edge(id, row * 3 + (col + 1));
            }
            // top neighbor: next row, same column
            if (row < 2)
            {
                rg.add_edge(id, (row + 1) * 3 + col);
            }
        }
    }

    // run A* once, up front — if no route exists there's nothing to simulate
    auto path = a_star(rg, 0, 8);
    if (!path.has_value())
    {
        std::cerr << "A* found no path\n";
        return 1;
    }

    // Spawn the car at the route's first waypoint, already facing the second one,
    // so it only needs small steering corrections instead of a hard U-turn on frame 1.
    const Point & start = path->at(0);
    double init_heading = 0.0;  // default: face +x (used when the route is a single point)
    if (path->size() >= 2)
    {
        // heading = angle of the vector from the first waypoint to the second.
        // atan2(dy, dx) handles all four quadrants (unlike atan(dy/dx)).
        init_heading = std::atan2(path->at(1).y - start.y, path->at(1).x - start.x);
    }
    KinematicModel car{Pose{start.x, start.y, init_heading}};

    RefLine rline = path_to_ref_line(*path);
    FrenetConfig frenetconfig;
    CostWeights weights;

    // Obstacles sitting just BESIDE the route (offset ~1.5m from the centerline)
    // so the planner only needs a modest swerve to clear them, not a max-width lurch.
    // Route runs up the left column (node 0->3), across the middle row (3->4->5),
    // then up the right column (5->8).
    std::vector<Obstacle> obstacles{
        Obstacle{Point{-4.5, -3.0}, 1.0},  // just east of the left column (0->3 segment)
        Obstacle{Point{ 3.0,  1.5}, 1.0},  // just north of the middle row (4->5 segment)
    };

    while (true)
    {
        // canvas: height x width, 3-channel BGR, white background
        canvas.setTo(cv::Scalar(255, 255, 255));  // clear to white each frame
        
        Point car_pos{car.pose().x, car.pose().y};
        FrenetPoint car_f = to_frenet(car_pos, rline);
        FrenetState fs = FrenetState(car_f.s, car.speed(), 0.0, car_f.d, 0.0, 0.0);
        
        double dist = dist_to_nearest_obstacle(car_pos, obstacles);
        State state = decide_state(dist);
        switch (state)
        {
            case State::CRUISE:
                frenetconfig.target_speed = 5.0;
                break;
            case State::SLOW:
                frenetconfig.target_speed = 2.5;
                break;
            case State::STOP:
                frenetconfig.target_speed = 0.0;
                break;
        }

        auto trajs = generate_frenet_trajectories(fs, rline, frenetconfig);

        double lowest_traj_cost = compute_cost(trajs[0], weights, frenetconfig);
        const FrenetTrajectory* best_traj = &trajs[0];

        for (const auto & traj : trajs)
        {
            if (is_collision(obstacles, traj, frenetconfig.car_radius)) continue;

            double cur_traj_cost = compute_cost(traj, weights, frenetconfig);
            if (cur_traj_cost < lowest_traj_cost)
            {
                lowest_traj_cost = cur_traj_cost;
                best_traj = &traj;
            }
        }

        draw_trajectories(trajs, canvas);
        draw_traj(*best_traj, canvas, cv::Scalar(0, 0, 255), 3);  // best = bold red
        draw_road_nodes(rg, canvas);
        draw_astar_path(*path, canvas);

        follow_trajectory(*best_traj, car);

        draw_obstacles(obstacles, canvas);
        draw_car(canvas, car);

        cv::imshow("window title", canvas);

        // 'q' to quit
        if (cv::waitKey(30) == 'q') { break; }
    }

    return 0;
}