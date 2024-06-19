#ifndef TUW_EIGEN__LINESEGMENT2D_HPP
#define TUW_EIGEN__LINESEGMENT2D_HPP

#include <tuw_eigen/utils.hpp>
#include <tuw_eigen/line2d.hpp>

namespace tuw_eigen
{
  class LineSegment2D; /// Prototype
  using LineSegment2DPtr = std::shared_ptr<LineSegment2D>;
  using LineSegment2DConstPtr = std::shared_ptr<LineSegment2D const>;

  /**
   * class to represent a 2D line as equation a*x + b*y + c = 0
   * The line has no endpoints
   **/
  class LineSegment2D : public Line2D
  {
  protected:
    Point2D p0_, p1_; /// the lines endpoints

  public:
    /**
     * constructor to create a line from points
     * @param p0
     * @param p1
     **/
    LineSegment2D(const Point2D &p0, const Point2D &p1);

    /**
     * sets a line from points
     * @param p0
     * @param p1
     **/
    LineSegment2D &set(const Point2D &p0, const Point2D &p1);
    /**
     * @return startpoint x
     **/
    const double &x0() const;
    /**
     * @return startpoint y
     **/
    const double &y0() const;
    /**
     * @return endpoint x
     **/
    const double &x1() const;
    /**
     * @return endpoint y
     **/
    const double &y1() const;

    /**
     * @return length of the line
     **/
    double length() const;

    /**
     * @return center point between p0 and p1
     **/
    Point2D pc() const;
    /**
     * @return the line function of the base class
     **/
    const Line2D &line() const;

    /**
     * orientation of the line in space
     * @retun angle between -PI and PI
     **/
    double angle() const;

    /**
     * comparison operator
     * @return true on equal
     **/
    bool operator==(const LineSegment2D &o) const;

    /** computes distance to line segment
     * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * @param p point
     * @param dx vector to point x
     * @param dx vector to point y
     * @return distance to line between the segment endpoints or the distance to the nearest endpoints
     **/
    double distanceSqrTo(const Point2D &p, double &dx, double &dy) const;

    /** computes distance to line segment
     * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * @param p point
     * @param dx vector to point x
     * @param dx vector to point y
     * @return distance to line between the segment endpoints or the distance to the nearest endpoints
     **/
    double distanceTo(const Point2D &p, double &dx, double &dy) const;

    /** computes distance to line segment
     * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * @return distance to line between the segment endpoints or the distance to the nearest endpoints
     **/
    double distanceTo(const Point2D &p) const;

    /** computes a ratio where a closest point along the segment is
     * @param p point
     * @return ratio of the segment length
     * @see closestPointLineSegmentRatio
     **/
    double closestPointLineSegmentRatio(const Point2D &p) const;
    /** computes closest point along segment
     * @param p point
     * @return closest point along segment to the given point
     **/
    Point2D closestPointTo(const Point2D &p) const;

  private:
    void recompute_line_equation();
  };

  using LineSegments2D = std::vector<LineSegment2D>;
  using LineSegments2DPtr = std::shared_ptr<LineSegment2D>;
  using LineSegments2DConstPtr = std::shared_ptr<LineSegment2D const>;
} // namespace tuw_eigen
#endif // TUW_EIGEN__LINESEGMENT2D_HPP
