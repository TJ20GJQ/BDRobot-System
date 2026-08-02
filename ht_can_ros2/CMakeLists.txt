cmake_minimum_required(VERSION 3.8)
project(ht_can_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(BITNESS "64bit")
    message(STATUS "architecture is 64 bits")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(BITNESS "32bit")
    message(STATUS "architecture is 32 bits")
else()
    message(FATAL_ERROR "Unsupported bitness: ${CMAKE_SIZEOF_VOID_P}")
endif()

# linux system use [uname -p] get CMAKE_SYSTEM_PROCESSOR
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
    message(STATUS "i386 or i686 architecture detected")
    set(ARCH_DIR "x86")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message(STATUS "x86_64 architecture detected")
    set(ARCH_DIR "x86")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
    message(STATUS "armv7l architecture detected")
    set(ARCH_DIR "arm")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(STATUS "aarch64 architecture detected")
    set(ARCH_DIR "arm")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(CAN_LIBRARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH_DIR}/${BITNESS}/libcontrolcan.so")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_ros2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveSdfzError.msg"
  "msg/Frame.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(ht_can_ros2_node
  src/ht_can_ros2_node.cpp
  src/ht_can_control.cpp
)

ament_target_dependencies(ht_can_ros2_node
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  std_msgs
  robot_ros2_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

target_link_libraries(ht_can_ros2_node
  "${cpp_typesupport_target}"
  ${CAN_LIBRARY_PATH}
)

target_include_directories(ht_can_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ht_can_ros2_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS ht_can_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ${CAN_LIBRARY_PATH}
  DESTINATION lib/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
