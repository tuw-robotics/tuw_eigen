#ifndef TUW_EIGEN__POINT2D_HPP
#define TUW_EIGEN__POINT2D_HPP

#include <tuw_eigen/utils.hpp>

namespace tuw_eigen
{
   class Point2D; /// Prototype
   using Point2DPtr = std::shared_ptr<Point2D>;
   using Point2DConstPtr = std::shared_ptr<Point2D const>;

   /**
    * class to represent a point
    * using homogeneous coordinates [x, y, 1]
    **/
   class Point2D : public Vector3d
   {
   public:
      friend std::ostream &operator<<(std::ostream &os, const Point2D &o)
      {
         os << "[" << o.x() << ", " << o.y() << "]";
         return os;
      }
      using Vector3d::Vector3d;

      Point2D()
          : Vector3d(0.0, 0.0, 1.0)
      {
      }
      Point2D(double x, double y)
          : Vector3d(x, y, 1.0)
      {
      }
      /**
       * translational x component
       * @return rotation
       **/
      const double &x() const
      {
         return (*this)[0];
      }
      /**
       * translational x component
       * @return rotation
       **/
      double &x()
      {
         return (*this)[0];
      }
      /**
       * translational y component
       * @return y component
       **/
      const double &y() const
      {
         return (*this)[1];
      }
      /**
       * translational y component
       * @return y component
       **/
      double &y()
      {
         return (*this)[1];
      }
      /**
       * homogeneous component
       * @return rotation
       **/
      const double &h() const
      {
         return (*this)[2];
      }
      /**
       * homogeneous component
       * @return rotation
       **/
      double &h()
      {
         return (*this)[2];
      }
      /**
       * returns the distance to an other point
       * @return disance
       **/
      double distanceTo(const Point2D &p) const
      {
         double dx = p[0] - (*this)[0], dy = p[1] - (*this)[1];
         return std::sqrt(dx * dx + dy * dy);
      }
      /**
       * returns the distance to an other point
       * @return disance
       **/
      double distanceTo(double x, double y) const
      {
         double dx = x - (*this)[0], dy = y - (*this)[1];
         return std::sqrt(dx * dx + dy * dy);
      }
      /**
       * angle form origin to point (alpha in polar space)
       * @see radius
       * @see Polar2D
       * @return angle between -PI and +PI
       **/
      double angle() const
      {
         return std::atan2((*this)[1], (*this)[0]);
      }
      /**
       * distance to origin (rho in polar space)
       * @see angle
       * @see Polar2D
       * @return distance
       **/
      double radius() const
      {
         return std::sqrt((*this)[0] * (*this)[0] + (*this)[1] * (*this)[1]);
      }
      /**
       * checks if a point is within a rectangle
       * @param x0 top left x
       * @param y0 top left y
       * @param x1 bottom right x
       * @param y1 bottom right y
       * @return true if inside
       **/
      bool inside(double x0, double y0, double x1, double y1) const
      {
         return ((*this)[0] >= x0) && ((*this)[0] <= x1) && ((*this)[1] >= y0) && ((*this)[1] <= y1);
      }
      /**
       * returns x and y as formated string
       * @param format using printf format
       * @return string
       **/
      std::string str(const char *format) const
      {
         char str[0xFF];
         sprintf(str, format, x(), y());
         return std::string(str);
      }
      /**
       * compares with within tolerance
       * be aware that this is a 3x1 vector and h will also be compared
       * @param o
       * @param tolerance
       **/
      bool equal(const Point2D &o, double tolerance) const
      {
         double d = (o - *this).norm();
         return d < tolerance;
      }
   };
   using Points2D = std::vector<Point2D>;
   using Points2DPtr = std::shared_ptr<Points2D>;
   using Points2DConstPtr = std::shared_ptr<Points2D const>;
} // namespace tuw

#endif // TUW_EIGEN__POINT2D_HPP
