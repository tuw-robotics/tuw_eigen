#ifndef TUW_EIGEN__LINESEGMENT2D_HPP
#define TUW_EIGEN__LINESEGMENT2D_HPP

#include <tuw_eigen/utils.hpp>
#include <tuw_eigen/point2d.hpp>

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
  Point2D p0_, p1_;  /// the lines endpoints

  public:

  
    /**
       * constructor to create a line from points
       * @param p0
       * @param p1
       **/
    LineSegment2D(const Point2D & p0, const Point2D & p1);

    
  /**
     * sets a line from points
     * @param p0
     * @param p1
     **/
  LineSegment2D & set(const Point2D & p0, const Point2D & p1);
  };

  using LineSegments2D = std::vector<LineSegment2D>;
  using LineSegments2DPtr = std::shared_ptr<LineSegment2D>;
  using LineSegments2DConstPtr = std::shared_ptr<LineSegment2D const>;
} // namespace tuw_eigen
#endif // TUW_EIGEN__LINESEGMENT2D_HPP
