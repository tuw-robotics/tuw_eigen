cmake_minimum_required(VERSION 3.8)
project(tuw_eigen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/line2d.cpp
  src/${PROJECT_NAME}/point2d.cpp
)

target_compile_features(tuw_eigen PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(tuw_eigen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  "Eigen3"
)

  # Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tuw_eigen PRIVATE "TUW_GEOMETRY_BUILDING_LIBRARY")

add_subdirectory(src/examples)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS tuw_eigen
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_eigen test/test_eigen.cpp)  # Need to link .cpp file under test
  target_include_directories(test_eigen PUBLIC src/)  # Need to specify directories in order to #include from them in the test
  target_link_libraries(test_eigen tuw_eigen)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  install(
    TARGETS test_eigen
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  tuw_eigen
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  "Eigen3"
)

ament_package()
