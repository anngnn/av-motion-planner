#include <opencv2/opencv.hpp>

#include "vehicle/kinematic_model.hpp"
#include "planning/road_graph.hpp"
#include "planning/astar.hpp"

constexpr double kScale = 50.0; // pixels per meter
constexpr int kWidth = 800;
constexpr int kHeight = 600;
constexpr double kHeadingLineLen = 20.0;

double world_to_pix(double coord, bool is_x)
{
    if (is_x)
    {
        return coord * kScale + kWidth / 2;

    }
    return -coord * kScale + kHeight / 2;    // origin at top-left, y increases downward
}

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

int main()
{
    // start at origin, facing +x (theta = 0)
    KinematicModel car{Pose{0.0, 0.0, 0.0}};

    cv::Mat canvas(kHeight, kWidth, CV_8UC3);
    
    Path waypoints{Point{-5.0, 5.0}, Point{5.0, 5.0}, Point{5.0, -5.0}, Point{-5.0, -5.0}};

    // build a road network
    RoadGraph rg;
    int id = 0;
    for (const auto& p : waypoints)
    {
        rg.add_node(id, p);
        ++id;
    }

    rg.add_edge(0,1);
    rg.add_edge(0,2);
    rg.add_edge(0,3);
    rg.add_edge(2,1);
    rg.add_edge(2,3);
    // run A*
    auto path = a_star(rg, 0, 2);

    while (true)
    {
        // canvas: height x width, 3-channel BGR, white background
        canvas.setTo(cv::Scalar(255, 255, 255));  // clear to white each frame

        // car.update(0.5, 0.1, 0.05);
        // auto car_pix_x = world_to_pix(car.pose().x, true);
        // auto car_pix_y = world_to_pix(car.pose().y, false);

        // cv::circle(canvas, cv::Point(car_pix_x, car_pix_y), 8, cv::Scalar(255, 0, 0), -1);
        
        // cv::Point p1(car_pix_x, car_pix_y);
        // auto end_x = car_pix_x + std::cos(car.pose().theta) * kHeadingLineLen;
        // auto end_y = car_pix_y - std::sin(car.pose().theta) * kHeadingLineLen;
        // cv::Point p2(end_x, end_y);

        // int thickness = 2;

        // cv::line(canvas, p1, p2, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA);

        draw_road_nodes(rg, canvas);

        cv::imshow("window title", canvas);

        // 'q' to quit
        if (cv::waitKey(30) == 'q') { break; }
    }

    return 0;
}