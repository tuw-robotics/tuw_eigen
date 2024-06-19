
#include <cmath>
#include <iostream>
#include "tuw_eigen/utils.hpp"
#include "tuw_eigen/linesegment2d.hpp"

using namespace tuw_eigen;

    LineSegment2D::LineSegment2D(const Point2D & p0, const Point2D & p1){
        :p0_(p0), p1_(p1)
    }


LineSegment2D & LineSegment2D::set(const Point2D & p0, const Point2D & p1)
{
  Line2D::set(p0, p1);
  p0_ = p0, p1_(p1);
  return *this;
}