// Copyright 2022 Markus Bader
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Markus Bader nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"
#include "tuw_eigen/eigen.hpp"

TEST(Pose3D, constructor)
{
  Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  Eigen::Vector3d translation(44.0, 33.0, 11.0);
  transform.translate(translation);
  tuw_eigen::Pose3D pose(44, 33, 11);
  std::cout << "Initial transformation matrix:\n" << transform.matrix() << std::endl;
  std::cout << "Initial transformation matrix:\n" << pose.matrix() << std::endl;
  ASSERT_EQ(pose.x(), transform.translation().x());
  ASSERT_EQ(pose.y(), transform.translation().y());
  ASSERT_EQ(pose.z(), transform.translation().z());
}
TEST(Pose3D, transation)
{
  Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  Eigen::Vector3d translation(2.0, 3.0, 12.0);
  transform.translate(translation);
  tuw_eigen::Pose3D pose = tuw_eigen::Pose3D::Identity();
  pose.x() = 2.0;
  pose.y() = 3.0;
  pose.z() = 12.0;
  //std::cout << "Initial transformation matrix:\n" << transform.matrix() << std::endl;
  //std::cout << "Initial transformation matrix:\n" << pose.matrix() << std::endl;
  ASSERT_EQ(pose.x(), transform.translation().x());
  ASSERT_EQ(pose.y(), transform.translation().y());
  ASSERT_EQ(pose.z(), transform.translation().z());
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}