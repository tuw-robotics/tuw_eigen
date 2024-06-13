#ifndef TUW_EIGEN__LINE2D_HPP
#define TUW_EIGEN__LINE2D_HPP

#include <cstdio>
#include <vector>
#include <memory>

namespace tuw_eigen
{
class Line2D;  /// Prototype
using Line2DPtr = std::shared_ptr<Line2D>;
using Line2DConstPtr = std::shared_ptr<Line2D const>;

/**
 * class to represent a 2D line as equation a*x + b*y + c = 0
 * The line has no endpoints
 **/
class Line2D
{
public:
  Line2D();
};
using Lines2D = std::vector<Line2D>;
using Lines2DPtr = std::shared_ptr<Lines2D>;
using Lines2DConstPtr = std::shared_ptr<Lines2D const>;
}  // namespace tuw_eigen
#endif  // TUW_EIGEN__LINE2D_HPP
