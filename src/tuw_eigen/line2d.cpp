
#include <cmath>
#include <iostream>
#include "tuw_eigen/utils.hpp"
#include "tuw_eigen/line2d.hpp"

using namespace tuw_eigen;

Line2D::Line2D() : Vector3d(){};
Line2D::Line2D(double a, double b, double c) : Vector3d(a, b, c){};
Line2D::Line2D(const Vector3d &eq) : Vector3d(eq){};
Line2D::Line2D(const Point2D & p0, const Point2D & p1)
{
    set(p0, p1);
}
Line2D::Line2D(double x0, double y0, double x1, double y1)
{
    set(x0, y0, x1, y1);
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

Line2D &Line2D::set(double x0, double y0, double x1, double y1)
{
    compute_line_equation(x0, y0, x1, y1, (*this));
    return (*this);
}
Line2D &Line2D::set(const Point2D & p0, const Point2D & p1)
{
    compute_line_equation(p0.x(), p0.y(), p1.x(), p1.y(), (*this));
    return (*this);
}

void Line2D::normalize()
{
    double r = std::hypot((*this)[0], (*this)[1]);
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

Point2D Line2D::pointOnLine(const Point2D &p) const
{
    return this->pointOnLine(p.x(), p.y());
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