#ifndef TUW_EIGEN__POSE3D_HPP
#define TUW_EIGEN__POSE3D_HPP

#include <tuw_eigen/utils.hpp>

namespace tuw_eigen
{
   class Pose3D; /// Prototype
   using Pose3DPtr = std::shared_ptr<Pose3D>;
   using Pose3DConstPtr = std::shared_ptr<Pose3D const>;

   /**
    * class to represent a transform in 3D space
    **/
   class Pose3D : public Eigen::Transform<double, 3, Eigen::Affine>
   {
   public:
      using Eigen::Transform<double, 3, Eigen::Affine>::Transform;
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
       * Angles around x, y, z
       * @param format the Euler-angles of the rotation matrix
       **/
      Vector3d eulerAngles() const;
   };
   using Poses3D = std::vector<Pose3D>;
   using Poses3DPtr = std::shared_ptr<Poses3D>;
   using Poses3DConstPtr = std::shared_ptr<Poses3D const>;
} // namespace tuw

#endif // TUW_EIGEN__POSE3D_HPP
