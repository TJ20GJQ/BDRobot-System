cmake_minimum_required(VERSION 3.8)
project(bdrobot_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)  # 必须加！且包名完全正确
find_package(moveit_core REQUIRED)            # MoveIt2核心库（必须加）
find_package(graph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
# find_package(trac_ik_lib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(action_files
    "action/ExecuteGoals.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${action_files}
    DEPENDENCIES action_msgs std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(bdrobot_arm_executor src/bdrobot_arm_executor.cpp)
target_include_directories(bdrobot_arm_executor PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_compile_features(bdrobot_arm_executor PUBLIC
c_std_99
cxx_std_17
)
target_link_libraries(bdrobot_arm_executor ${cpp_typesupport_target})
ament_target_dependencies(bdrobot_arm_executor
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2
  tf2_geometry_msgs
)

## ==============================================================================

## ==============================================================================

# 安装Python脚本（关键：将scripts目录下的.py文件复制到安装目录）
install(PROGRAMS
  scripts/moveit2_cartesian_control.py  # 你的Python代码路径
  DESTINATION lib/${PROJECT_NAME}       # 安装路径（和C++节点同目录）
)

# 添加可执行文件：fk_demo
add_executable(fk_demo src/fk_demo.cpp)
target_include_directories(fk_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # ${OMPL_INCLUDE_DIRS}
)
target_compile_features(fk_demo PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(fk_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

# 添加可执行文件：ik_demo
add_executable(ik_demo src/ik_demo.cpp)
target_include_directories(ik_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ik_demo PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(ik_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

# 添加可执行文件：move_cartesian_demo
add_executable(move_cartesian_demo src/move_cartesian_demo.cpp)
target_include_directories(move_cartesian_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(move_cartesian_demo PUBLIC 
  c_std_99 
  cxx_std_17
)
ament_target_dependencies(move_cartesian_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

# 安装可执行文件
install(TARGETS
  fk_demo
  ik_demo
  move_cartesian_demo
  bdrobot_arm_executor
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

# output include
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

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
