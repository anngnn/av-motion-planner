#include "planning/frenet.hpp"
#include "common/math_utils.hpp"

RefLine path_to_refline(const Path & path)
{
    RefLine refline;
    double running_s = 0;
    double heading = 0.0;
    if (path.size() < 2) { return refline; }
    for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
    {
        const Point& cur  = path.at(i);
        const Point& next  = path.at(i + 1);
        heading = std::atan2(next.y - cur.y, next.x - cur.x);
        refline.push_back(RefPoint{Pose{cur.x, cur.y, heading}, running_s});
        running_s += eucl_dist(cur, next);
    }
    refline.push_back(RefPoint{Pose{path.back().x, path.back().y, heading}, running_s});
    return refline;
}