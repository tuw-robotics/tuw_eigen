
#include <cmath>
#include <iostream>
#include "tuw_eigen/utils.hpp"
#include "tuw_eigen/linesegment2d.hpp"

using namespace tuw_eigen;

LineSegment2D::LineSegment2D(const Point2D &p0, const Point2D &p1)
    : Line2D(p0, p1), p0_(p0), p1_(p1)
{
}

LineSegment2D::LineSegment2D(double x0, double y0, double x1, double y1)
: Line2D(x0, y0, x1, y1), p0_(x0, y0), p1_(x1, y1)
{
}
const double & LineSegment2D::x0() const {return p0_.x();}
const double & LineSegment2D::y0() const {return p0_.y();}
const double & LineSegment2D::x1() const {return p1_.x();}
const double & LineSegment2D::y1() const {return p1_.y();}

LineSegment2D &LineSegment2D::set(const Point2D &p0, const Point2D &p1)
{
  Line2D::set(p0, p1);
  p0_ = p0, p1_(p1);
  return *this;
}

void LineSegment2D::recompute_line_equation()
{
  Line2D::compute_line_equation(p0_.x(), p0_.y(), p1_.x(), p1_.y(), (*this));
}

double LineSegment2D::length() const
{
  return p0_.distanceTo(p1_);
}
Point2D LineSegment2D::pc() const
{
  double dx = p1_.x() - p0_.x();
  double dy = p1_.y() - p0_.y();
  return Point2D(p0_.x() + dx / 2., p0_.y() + dy / 2.);
}

const Line2D &LineSegment2D::line() const
{
  return *this;
}

double LineSegment2D::angle() const
{
  double dx = p1_.x() - p0_.x();
  double dy = p1_.y() - p0_.y();
  return atan2(dy, dx);
}
bool LineSegment2D::operator==(const LineSegment2D & o) const
{
  return p0_ == o.p0_ && p1_ == o.p1_;
}
/** computes squared distance to line segment
 * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 * @param p point
 * @param dx vector to point x
 * @param dx vector to point y
 * @return distance to line between the segment endpoints or the distance to the nearest endpoints **/
double LineSegment2D::distanceSqrTo(const Point2D & p, double & dx, double & dy) const
{
  const double px = x1() - x0();
  const double py = y1() - y0();
  const double l2 = px * px + py * py;
  double u = ((p.x() - x0()) * px + (p.y() - y0()) * py) / l2;
  if (u > 1) {
    u = 1;
  } else if (u < 0) {
    u = 0;
  }
  const double xk = x0() + u * px;
  const double yk = y0() + u * py;
  dx = xk - p.x();
  dy = yk - p.y();
  return dx * dx + dy * dy;
}
/** computes distance to line segment
 * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 * @param p point
 * @param dx vector to point x
 * @param dx vector to point y
 * @return distance to line between the segment endpoints or the distance to the nearest endpoints **/
double LineSegment2D::distanceTo(const Point2D & p, double & dx, double & dy) const
{
  return std::sqrt(distanceSqrTo(p, dx, dy));
}
/** computes distance to line segment
 * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 * @param p point
 * @return distance to line between the segment endpoints or the distance to the nearest endpoints **/
double LineSegment2D::distanceTo(const Point2D & p) const
{
  double dx, dy;
  return distanceTo(p, dx, dy);
}

double LineSegment2D::closestPointLineSegmentRatio(const Point2D & p) const
{
  double px = x1() - x0();
  double py = y1() - y0();
  double l2 = px * px + py * py;
  double u = ((p.x() - x0()) * px + (p.y() - y0()) * py) / l2;
  if (u > 1) {
    u = 1;
  } else if (u < 0) {
    u = 0;
  }
  return u;
}

Point2D LineSegment2D::closestPointTo(const Point2D & p) const
{
  double px = x1() - x0();
  double py = y1() - y0();
  const double u = closestPointLineSegmentRatio(p);
  return Point2D(x0() + u * px, y0() + u * py);
}