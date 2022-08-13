#ifndef MATH_LIB_H
#define MATH_LIB_H

#include <opencv2/opencv.hpp>

double cross_product(cv::Point2d& p1, cv::Point2d& p2)
{
    return p1.x*p2.y-p2.x*p1.y;
}

double product(cv::Point2d& p1, cv::Point2d& p2)
{
    return p1.x*p2.x + p1.y*p2.y;
}

cv::Point2d calc_normal(cv::Point2d dir)
{
    double len = std::sqrt(dir.x*dir.x + dir.y*dir.y);
    return cv::Point2d(-dir.y/len, dir.x/len);
}

#endif
