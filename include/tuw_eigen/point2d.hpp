#ifndef TUW_EIGEN__POINT2D_HPP
#define TUW_EIGEN__POINT2D_HPP

#include <tuw_eigen/utils.hpp>
#include <tuw_eigen/geometry_msgs.hpp>

namespace tuw_eigen
{
   class Point2D; /// Prototype
   using Point2DPtr = std::shared_ptr<Point2D>;
   using Point2DConstPtr = std::shared_ptr<Point2D const>;

   /**
    * class to represent a point
    * using homogeneous coordinates [x, y, 1]
    **/
   class Point2D : public Vector2d
   {
   public:
      friend std::ostream &operator<<(std::ostream &os, const Point2D &o)
      {
         os << "[" << o.x() << ", " << o.y() << "]";
         return os;
      }
      using Vector2d::Vector2d;

      /**
       * constructor to generate a point from a ros geometry_msgs point
       **/
      template<class ContainerAllocator>
      Point2D(const geometry_msgs::msg::Point_<ContainerAllocator> &p)
      : Vector2d(p.x, p.y)
      {}

      
      /**
       * copies x and y to a geometry_msgs point and leaves z untouched
       * @param ros geometry point object
       * @return point
       **/
      template<class ContainerAllocator>
      geometry_msgs::msg::Point_<ContainerAllocator> &copy_to(geometry_msgs::msg::Point_<ContainerAllocator> &p){
         p.x = this->x(), p.y = this->y();
         return p;
      }
      
      /**
       * copies x and y to a geometry_msgs point and sets z to 0
       * @param ros geometry point object
       * @return point
       **/
      template<class ContainerAllocator>
      geometry_msgs::msg::Point_<ContainerAllocator> &copy_to_clear(geometry_msgs::msg::Point_<ContainerAllocator> &p){
         p.x = this->x(), p.y = this->y(), p.z = 0;
         return p;
      }

      /**
       * returns the distance to an other point
       * @return disance
       **/
      double distanceTo(const Point2D &p) const;

      /**
       * returns the distance to an other point
       * @return disance
       **/
      double distanceTo(double x, double y) const;

      /**
       * angle form origin to point (alpha in polar space)
       * @see radius
       * @see Polar2D
       * @return angle between -PI and +PI
       **/
      double angle() const;

      /**
       * distance to origin (rho in polar space)
       * @see angle
       * @see Polar2D
       * @return distance
       **/
      double radius() const;

      /**
       * checks if a point is within a rectangle
       * @param x0 top left x
       * @param y0 top left y
       * @param x1 bottom right x
       * @param y1 bottom right y
       * @return true if inside
       **/
      bool inside(double x0, double y0, double x1, double y1) const;

      /**
       * returns x and y as formated string
       * @param format using printf format
       * @return string
       **/
      std::string str(const char *format) const;

      /**
       * compares with within tolerance
       * be aware that this is a 3x1 vector and h will also be compared
       * @param o
       * @param tolerance
       **/
      bool equal(const Point2D &o, double tolerance) const;

   };
   using Points2D = std::vector<Point2D>;
   using Points2DPtr = std::shared_ptr<Points2D>;
   using Points2DConstPtr = std::shared_ptr<Points2D const>;
} // namespace tuw

#endif // TUW_EIGEN__POINT2D_HPP
