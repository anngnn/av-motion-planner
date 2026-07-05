#include <iostream>
#include <opencv2/opencv.hpp>

#include "vehicle/kinematic_model.hpp"
#include "planning/road_graph.hpp"
#include "planning/astar.hpp"
#include "common/math_utils.hpp"

constexpr double kScale = 30.0; // pixels per meter
constexpr int kWidth = 1000;
constexpr int kHeight = 1000;
constexpr double kHeadingLineLen = 20.0;
constexpr double kReachThreshold = 1.5;

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
// steer toward it, then if we're within kReachThreshold, retarget the next one.
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
    if ( eucl_dist(Point{car.pose().x, car.pose().y}, goal_wp)< kReachThreshold)
    {
        wp_id++;
    }

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

    int wp_id = 0;

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

    while (true)
    {
        // canvas: height x width, 3-channel BGR, white background
        canvas.setTo(cv::Scalar(255, 255, 255));  // clear to white each frame

        draw_road_nodes(rg, canvas);
        draw_astar_path(*path, canvas);

        if (wp_id < static_cast<int>(path->size()))
        {
            step_toward_waypoint(*path, wp_id, car);
        }

        draw_car(canvas, car);

        cv::imshow("window title", canvas);

        // 'q' to quit
        if (cv::waitKey(30) == 'q') { break; }
    }

    return 0;
}