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
    /**
     * constructor 
     **/
    Line2D();

    /**
     * constructor  to create a line from equation
     * @param a
     * @param b
     * @param c
     **/
    Line2D(double a, double b, double c);

    /**
     * constructor  to create a line from equation
     * @param eq [a, b, c]
     **/
    Line2D(const Vector3d& eq);

    /**
     * constructor to create a line from points
     * and normalizes equation
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param normalize normalizes equation on true
     **/
    Line2D(double x0, double y0, double x1, double y1);
    
    /**
     * constructor to create a line from points
     * and normalizes equation
     * @param p0
     * @param p1
     **/
    Line2D(const Point2D & p0, const Point2D & p1);

    /**
     * @return the first equation component for x
     **/
    const double &a() const;
    
    /**
     * @return the second equation component for y
     **/
    double &b();
    
    /**
     * @return the second equation component for y
     **/
    const double &b() const;
    
    /**
     * @return the third equation component
     **/
    double &c();
    
    /**
     * @return the third equation component
     **/
    const double &c() const;

    /**
     * constructor to create a line from points
     * and normalizes equation
     * @param p0
     * @param p1
     * @return ref to this
     **/
    Line2D &set(const Point2D & p0, const Point2D & p1);

    /**
     * constructor to create a line from points
     * and normalizes equation
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @return ref to this
     **/
    Line2D &set(double x0, double y0, double x1, double y1);

    /**
     * normalizes the equation to a*a + b*b = 1
     **/
    void normalize();

    /**
     * computes the distance to a point
     * @param x
     * @param y
     * @returns the minimal distance to a point
     * @pre normalize
     **/
    double distanceTo(double x, double y) const;

    /**
     * computes a point on the line with the shortest distance to the point given
     * @param x
     * @param y
     * @returns point on line
     * @pre normalize
     **/
    Point2D pointOnLine(double x, double y) const;
    /**
     * computes a point on the line with the shortest distance to the point given
     * @param p
     * @returns point on line
     * @pre normalize
     **/
    Point2D pointOnLine(const Point2D &p) const;

    /**
     * computes the intersection point of two lines
     * @param l
     * @returns point on line
     * @pre normalize
     **/
    Point2D intersection(const Line2D &l) const;

    /**
     * restuns the normal vector to a line
     * @return vector
     **/
    Eigen::Vector2d normal() const;

    /**
     * restuns the direction vector to a line
     * @return vector
     **/
    Eigen::Vector2d direction() const;
    
    
    template<typename T>
    static void compute_line_equation(const T &x0, const T &y0, const T &x1, const T &y1, Eigen::Matrix<T, 3, 1> &des){
        des[0] = y0 - y1;
        des[1] = x1 - x0,
        des[2] = x0 * y1 - y0 * x1; /// cross product with homogenios vectors
        T r = std::hypot(des[0], des[1]);
        des[0] /= r, des[1] /= r, des[2] /= r;
    }
  };

  using Lines2D = std::vector<Line2D>;
  using Lines2DPtr = std::shared_ptr<Lines2D>;
  using Lines2DConstPtr = std::shared_ptr<Lines2D const>;
} // namespace tuw_eigen
#endif // TUW_EIGEN__LINE2D_HPP
