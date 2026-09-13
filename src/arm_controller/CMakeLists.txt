cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pinocchio REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(arm_model REQUIRED)

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

pluginlib_export_plugin_description_file(controller_interface plugin.xml)

add_library(arm_controller SHARED
  controller/src/controller.cpp
  arm/arm_fsm.cpp
  arm/default6dof_task.cpp
  arm/model_from_urdf.cpp
  lib/calculate/arm.cpp
  lib/calculate/kinamic.cpp
  lib/calculate/trajectory.cpp
)

target_include_directories(arm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/arm>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/calculate>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/fsm>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  arm_controller
  rclcpp
  controller_interface
  pluginlib
  hardware_interface
  robot_msgs
  ament_index_cpp
  kdl_parser
  orocos_kdl
)

target_link_libraries(arm_controller
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS arm_controller
LIBRARY DESTINATION lib)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY controller/inc/
  DESTINATION include)

install(DIRECTORY arm/
  DESTINATION include/arm_controller
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY lib/calculate/
  DESTINATION include/arm_controller
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY lib/fsm/
  DESTINATION include/arm_controller
  FILES_MATCHING PATTERN "*.hpp")

ament_package()
