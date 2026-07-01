#include <opencv2/opencv.hpp>

#include "vehicle/kinematic_model.hpp"

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

int main()
{
    // start at origin, facing +x (theta = 0)
    KinematicModel car{Pose{0.0, 0.0, 0.0}};

    cv::Mat canvas(kHeight, kWidth, CV_8UC3);
    while (true)
    {
        // canvas: height x width, 3-channel BGR, white background
        canvas.setTo(cv::Scalar(255, 255, 255));  // clear to white each frame

        car.update(0.5, 0.1, 0.05);
        auto car_pix_x = world_to_pix(car.pose().x, true);
        auto car_pix_y = world_to_pix(car.pose().y, false);

        cv::circle(canvas, cv::Point(car_pix_x, car_pix_y), 8, cv::Scalar(255, 0, 0), -1);
        
        cv::Point p1(car_pix_x, car_pix_y);
        auto end_x = car_pix_x + std::cos(car.pose().theta) * kHeadingLineLen;
        auto end_y = car_pix_y - std::sin(car.pose().theta) * kHeadingLineLen;
        cv::Point p2(end_x, end_y);

        int thickness = 2;

        cv::line(canvas, p1, p2, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA);

        cv::imshow("window title", canvas);

        // waitKey(ms) â pauses for ms, returns key pressed (-1 if none)
        // 'q' to quit
        if (cv::waitKey(30) == 'q') { break; }
    }

    return 0;
}