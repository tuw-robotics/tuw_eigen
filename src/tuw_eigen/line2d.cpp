
#include <cmath>
#include <iostream>
#include "tuw_eigen/utils.hpp"
#include "tuw_eigen/line2d.hpp"

using namespace tuw_eigen;

Line2D::Line2D(double x0, double y0, double x1, double y1)
{
    set(x0, y0, x1, y1);
}

Line2D::Line2D(double x0, double y0, double x1, double y1, bool normalize)
{
    set(x0, y0, x1, y1, normalize);
}

const double &Line2D::a() const
{
    return (*this)[0];
}

double &Line2D::b()
{
    return (*this)[1];
}

const double &Line2D::b() const
{
    return (*this)[1];
}

double &Line2D::c()
{
    return (*this)[2];
}

const double &Line2D::c() const
{
    return (*this)[2];
}

Line2D &Line2D::set(double x0, double y0, double x1, double y1, bool normalize)
{
    (*this)[0] = y0 - y1, (*this)[1] = x1 - x0,
    (*this)[2] = x0 * y1 - y0 * x1; /// cross product with homogenios vectors
    if (normalize)
        this->normalize();
    return *this;
}

Line2D &Line2D::set(double x0, double y0, double x1, double y1)
{
    (*this)[0] = y0 - y1, (*this)[1] = x1 - x0,
    (*this)[2] = x0 * y1 - y0 * x1; /// cross product with homogenios vectors
    this->normalize();
    return *this;
}

void Line2D::normalize()
{
    double r = std::sqrt((*this)[0] * (*this)[0] + (*this)[1] * (*this)[1]);
    (*this)[0] /= r, (*this)[1] /= r, (*this)[2] /= r;
}

double Line2D::distanceTo(double x, double y) const
{
    return (*this)[0] * x + (*this)[1] * y + (*this)[2];
}

Point2D Line2D::pointOnLine(double x, double y) const
{
    double d = distanceTo(x, y);
    return Point2D(x - d * (*this)[0], y - d * (*this)[1]);
}

Point2D Line2D::intersection(const Line2D &l) const
{
    Vector3d h = this->cross(l);
    return Point2D(h[0] / h[2], h[1] / h[2]);
}

Eigen::Vector2d Line2D::normal() const
{
    return Eigen::Vector2d((*this)[0], (*this)[1]);
}

Eigen::Vector2d Line2D::direction() const
{
    return Eigen::Vector2d((*this)[1], -(*this)[0]);
}