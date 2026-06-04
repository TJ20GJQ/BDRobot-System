cmake_minimum_required(VERSION 3.8)
project(robot_self_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(robot_self_filter
  src/robot_self_filter.cpp
  src/robot_self_filter_node.cpp
)

target_include_directories(robot_self_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(robot_self_filter PUBLIC
  c_std_99
  cxx_std_17
)

ament_target_dependencies(robot_self_filter
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  livox_ros_driver2
  Eigen3
  sensor_msgs
  visualization_msgs
)

target_link_libraries(robot_self_filter
  yaml-cpp
  OpenMP::OpenMP_CXX
)

install(TARGETS robot_self_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
