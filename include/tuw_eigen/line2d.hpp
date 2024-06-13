#ifndef TUW_EIGEN__LINE2D_HPP
#define TUW_EIGEN__LINE2D_HPP

#include <tuw_eigen/utils.hpp>
#include <tuw_eigen/point2d.hpp>

namespace tuw_eigen
{
  class Line2D; /// Prototype
  using Line2DPtr = std::shared_ptr<Line2D>;
  using Line2DConstPtr = std::shared_ptr<Line2D const>;

  /**
   * class to represent a 2D line as equation a*x + b*y + c = 0
   * The line has no endpoints
   **/
  class Line2D : public Vector3d
  {
  public:
    friend std::ostream &operator<<(std::ostream &os, const Line2D &o)
    {
      os << "[" << o.a() << ", " << o.b() << ", " << o.c() << "]";
      return os;
    }
    using Vector3d::Vector3d;
    /**
     * constructor to create a line from points
     * and normalizes equation
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param normalize normalizes equation on true
     **/
    Line2D(double x0, double y0, double x1, double y1)
    {
      set(x0, y0, x1, y1);
    }
    /**
     * constructor to create a line from points
     * and normalizes equation optional
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param normalize normalizes equation on true
     **/
    Line2D(double x0, double y0, double x1, double y1, bool normalize)
    {
      set(x0, y0, x1, y1, normalize);
    }
    /**
     * @return the first equation component for x
     **/
    const double &a() const
    {
      return (*this)[0];
    }
    /**
     * @return the second equation component for y
     **/
    double &b()
    {
      return (*this)[1];
    }
    /**
     * @return the second equation component for y
     **/
    const double &b() const
    {
      return (*this)[1];
    }
    /**
     * @return the third equation component
     **/
    double &c()
    {
      return (*this)[2];
    }
    /**
     * @return the third equation component
     **/
    const double &c() const
    {
      return (*this)[2];
    }
    /**
     * constructor to create a line from points
     * and normalizes equation optional
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param normalize normalizes equation on true
     * @return ref to this
     **/
    Line2D &set(double x0, double y0, double x1, double y1, bool normalize)
    {
      (*this)[0] = y0 - y1, (*this)[1] = x1 - x0,
      (*this)[2] = x0 * y1 - y0 * x1; /// cross product with homogenios vectors
      if (normalize)
        this->normalize();
      return *this;
    }
    /**
     * constructor to create a line from points
     * and normalizes equation
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @return ref to this
     **/
    Line2D &set(double x0, double y0, double x1, double y1)
    {
      (*this)[0] = y0 - y1, (*this)[1] = x1 - x0,
      (*this)[2] = x0 * y1 - y0 * x1; /// cross product with homogenios vectors
      this->normalize();
      return *this;
    }
    /**
     * normalizes the equation to a*a + b*b = 1
     **/
    void normalize()
    {
      double r = std::sqrt((*this)[0] * (*this)[0] + (*this)[1] * (*this)[1]);
      (*this)[0] /= r, (*this)[1] /= r, (*this)[2] /= r;
    }
    /**
     * computes the distance to a point
     * @param x
     * @param y
     * @returns the minimal distance to a point
     * @pre normalize
     **/
    double distanceTo(double x, double y) const
    {
      return (*this)[0] * x + (*this)[1] * y + (*this)[2];
    }
    /**
     * computes a point on the line with the shortest distance to the point given
     * @param x
     * @param y
     * @returns point on line
     * @pre normalize
     **/
    Point2D pointOnLine(double x, double y) const
    {
      double d = distanceTo(x, y);
      return Point2D(x - d * (*this)[0], y - d * (*this)[1]);
    }
    /**
     * computes the intersection point of two lines
     * @param l
     * @returns point on line
     * @pre normalize
     **/
    Point2D intersection(const Line2D &l) const
    {
      Eigen::Vector3d h = this->cross(l);
      return Point2D(h[0] / h[2], h[1] / h[2]);
    }
    /**
     * restuns the normal vector to a line
     * @return vector
     **/
    Eigen::Vector2d normal() const
    {
      return Eigen::Vector2d((*this)[0], (*this)[1]);
    }
    /**
     * restuns the direction vector to a line
     * @return vector
     **/
    Eigen::Vector2d direction() const
    {
      return Eigen::Vector2d((*this)[1], -(*this)[0]);
    }
  };

  using Lines2D = std::vector<Line2D>;
  using Lines2DPtr = std::shared_ptr<Lines2D>;
  using Lines2DConstPtr = std::shared_ptr<Lines2D const>;
} // namespace tuw_eigen
#endif // TUW_EIGEN__LINE2D_HPP
