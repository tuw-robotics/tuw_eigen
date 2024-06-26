#ifndef TUW_EIGEN__UTILS_HPP
#define TUW_EIGEN__UTILS_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Geometry>

namespace tuw_eigen
{
  using Vector2d = Eigen::Matrix<double, 2, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Quaternion = Eigen::Quaterniond;
}  // namespace tuw_eigen

#endif  // TUW_EIGEN__UTILS_HPP
