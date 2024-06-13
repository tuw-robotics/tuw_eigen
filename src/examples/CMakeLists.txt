
add_executable(example main_example.cpp)
target_include_directories(example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(example
  rclcpp::rclcpp
  ${PROJECT_NAME}
)

install(TARGETS example
  DESTINATION lib/${PROJECT_NAME}
)
