
#include <cmath>
#include <iostream>
#include "tuw_eigen/point2d.hpp"

using namespace tuw_eigen;

double Point2D::distanceTo(const Point2D &p) const
{
    return distanceTo(p[0], p[1]);
}

double Point2D::distanceTo(double x, double y) const
{
    double dx = x - (*this)[0], dy = y - (*this)[1];
    return std::hypot(dx, dy);
}

double Point2D::angle() const
{
    return std::atan2((*this)[1], (*this)[0]);
}

double Point2D::radius() const
{
    return std::hypot((*this)[0], (*this)[1]);
}

bool Point2D::inside(double x0, double y0, double x1, double y1) const
{
    return ((*this)[0] >= x0) && ((*this)[0] <= x1) && ((*this)[1] >= y0) && ((*this)[1] <= y1);
}

std::string Point2D::str(const char *format) const
{
    char str[0xFF];
    sprintf(str, format, x(), y());
    return std::string(str);
}

bool Point2D::equal(const Point2D &o, double tolerance) const
{
    double d = (o - *this).norm();
    return d < tolerance;
}