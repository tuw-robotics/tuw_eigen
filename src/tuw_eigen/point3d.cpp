
#include <cmath>
#include <iostream>
#include "tuw_eigen/point3d.hpp"

using namespace tuw_eigen;

double Point3D::distanceTo(const Point3D &p) const
{
    return ((*this)-p).norm();
}

double Point3D::distanceTo(double x, double y, double z) const
{
    return distanceTo(Point3D(x, y, z));
}

bool Point3D::inside(double x0, double y0, double z0, double x1, double y1, double z1) const
{
    return ((*this)[0] >= x0) && ((*this)[0] <= x1) && ((*this)[1] >= y0) && ((*this)[1] <= y1) && ((*this)[2] >= z0) && ((*this)[2] <= z1);
}

std::string Point3D::str(const char *format) const
{
    char str[0xFF];
    sprintf(str, format, x(), y(), z());
    return std::string(str);
}

bool Point3D::equal(const Point3D &o, double tolerance) const
{
    double d = (o - *this).norm();
    return d < tolerance;
}