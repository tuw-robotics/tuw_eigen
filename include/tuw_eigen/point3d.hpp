#ifndef TUW_EIGEN__POINT3D_HPP
#define TUW_EIGEN__POINT3D_HPP

#include <tuw_eigen/utils.hpp>
#include <tuw_eigen/geometry_msgs.hpp>

namespace tuw_eigen
{
   class Point3D; /// Prototype
   using Point3DPtr = std::shared_ptr<Point3D>;
   using Point3DConstPtr = std::shared_ptr<Point3D const>;

   /**
    * class to represent a point [x, y, z]
    **/
   class Point3D : public Vector3d
   {
   public:
      friend std::ostream &operator<<(std::ostream &os, const Point3D &o)
      {
         os << "[" << o.x() << ", " << o.y() << ", " << o.z() << "]";
         return os;
      }
      using Vector3d::Vector3d;

      /**
       * returns the distance to an other point
       * @return disance
       **/
      double distanceTo(const Point3D &p) const;

      /**
       * returns the distance to an other point
       * @return disance
       **/
      double distanceTo(double x, double y, double z) const;

      /**
       * checks if a point is within a rectangle
       * @param x0 top left x
       * @param y0 top left y
       * @param x1 bottom right x
       * @param y1 bottom right y
       * @return true if inside
       **/
      bool inside(double x0, double y0, double z0, double x1, double y1, double z1) const;

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
      bool equal(const Point3D &o, double tolerance) const;
      
      
      /**
       * copies x and y to a geometry_msgs point and leaves z untouched
       * @param ros geometry point object
       * @return point
       **/
      template<class ContainerAllocator>
      geometry_msgs::msg::Point_<ContainerAllocator> &copy_from(geometry_msgs::msg::Point_<ContainerAllocator> &p){
         this->x() = p.x; 
         this->y() = p.y; 
         this->z() = p.z;
         return p;
      }
      
   };
   using Points3D = std::vector<Point3D>;
   using Points3DPtr = std::shared_ptr<Points3D>;
   using Points3DConstPtr = std::shared_ptr<Points3D const>;
} // namespace tuw

#endif // TUW_EIGEN__POINT3D_HPP
