#ifndef TUW_EIGEN__POSE3D_HPP
#define TUW_EIGEN__POSE3D_HPP

#include <tuw_eigen/point3d.hpp>
#include <tuw_eigen/utils.hpp>
#include <tuw_eigen/geometry_msgs.hpp>

namespace tuw_eigen
{
   class Pose3D; /// Prototype
   using Pose3DPtr = std::shared_ptr<Pose3D>;
   using Pose3DConstPtr = std::shared_ptr<Pose3D const>;

   /**
    * class to represent a pose in 3D space
    * Which is the inverse of the transform into that pose space
    **/
   class Pose3D : public Eigen::Transform<double, 3, Eigen::Affine>
   {
   public:
      /**
       * construtor
       * Copy all base constructors
       **/
      using Eigen::Transform<double, 3, Eigen::Affine>::Transform;

      /**
       * copy construtor
       * @param src
       **/
      Pose3D(const Eigen::Transform<double, 3, Eigen::Affine> &src);

      /**
       * copy construtor
       * @param src
       **/
      Pose3D(const Pose3D &src);

      /**
       * construtor
       * @param x translation
       * @param y translation
       * @param z translation
       **/
      Pose3D(double x, double y, double z);

      /**
       * computes the pose based on a position and a point to look at
       * @param position poistion
       * @param point_ahead
       **/
      Pose3D(const Point3D &position, const Point3D &point_ahead);

      /**
       * translational x component
       * @return rotation
       **/
      const double &x() const;

      /**
       * translational x component
       * @return rotation
       **/
      double &x();
      /**
       * translational y component
       * @return y component
       **/
      const double &y() const;

      /**
       * translational y component
       * @return y component
       **/
      double &y();
      /**
       * translational z component
       * @return rotation
       **/
      const double &z() const;

      /**
       * translational z component
       * @return rotation
       **/
      double &z();
      /**
       * returns x and y as formated string
       * @param format using printf format
       **/
      std::string str(const char *format) const;

      /**
       * Position
       * @return position
       **/
      Point3D position() const;

      /**
       * computes the pose based on a position and a point to look at
       * @param position poistion
       * @param point_ahead
       * @return this
       **/
      Pose3D &set(const Point3D &position, const Point3D &point_ahead);

      /**
       * Angles around x, y, z
       * @param format the Euler-angles of the rotation matrix
       **/
      Vector3d eulerAngles() const;


      template <class ContainerAllocator>
      geometry_msgs::msg::Pose_<ContainerAllocator> &set_translation(geometry_msgs::msg::Point_<ContainerAllocator> &p){
         this->translation() = Vector3d(p.x, p.y, p.z);
      }

      /**
       * copies the pose to a geometry_msgs object
       * @param ros geometry pose object
       * @return point
       **/
      template <class ContainerAllocator>
      geometry_msgs::msg::Pose_<ContainerAllocator> &copy_to(geometry_msgs::msg::Pose_<ContainerAllocator> &p)
      {
         p.position.x = this->x();
         p.position.y = this->y();
         p.position.z = this->z();

         // Extract the rotation part (3x3 matrix)
         Eigen::Matrix3d rotationMatrix = this->rotation();

         // Convert the rotation matrix to a quaternion
         Eigen::Quaterniond quaternion(rotationMatrix);

         p.orientation.x = quaternion.x();
         p.orientation.y = quaternion.y();
         p.orientation.z = quaternion.z();
         p.orientation.w = quaternion.w();
         return p;
      }      
      
      /**
       * copies the pose from a geometry_msgs object
       * @param ros geometry pose object
       * @return point
       **/
      template <class ContainerAllocator>
      Pose3D &copy_from(geometry_msgs::msg::Pose_<ContainerAllocator> &p)
      {
         this->x() = p.position.x;
         this->y() = p.position.y;
         this->z() = p.position.z;
         
         // Extract the rotation part (3x3 matrix)
         Eigen::Matrix3d rotationMatrix = this->rotation();

         // Convert the rotation matrix to a quaternion
         Eigen::Quaterniond quaternion( p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
         this->linear() = quaternion.toRotationMatrix();
         return p;
      }

      static const Pose3D Identity();
   };
   using Poses3D = std::vector<Pose3D>;
   using Poses3DPtr = std::shared_ptr<Poses3D>;
   using Poses3DConstPtr = std::shared_ptr<Poses3D const>;
} // namespace tuw

#endif // TUW_EIGEN__POSE3D_HPP
