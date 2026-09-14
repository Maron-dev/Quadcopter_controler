cmake_minimum_required(VERSION 3.8)
project(quadcopter_sim)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(gazebo_actuator SHARED src/gazebo_actuator.cpp)
target_compile_features(gazebo_actuator PUBLIC cxx_std_17)
target_include_directories(gazebo_actuator PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(gazebo_actuator ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_actuator
  gazebo_ros nav_msgs rclcpp std_msgs)

add_executable(motor_allocator src/motor_allocator.cpp)
target_compile_features(motor_allocator PUBLIC cxx_std_17)
ament_target_dependencies(motor_allocator geometry_msgs rclcpp std_msgs)

add_executable(quadcopter_controller src/quadcopter_controller.cpp)
target_compile_features(quadcopter_controller PUBLIC cxx_std_17)
ament_target_dependencies(quadcopter_controller geometry_msgs nav_msgs rclcpp)

add_executable(lqr_controller src/lqr_controller.cpp)
target_compile_features(lqr_controller PUBLIC cxx_std_17)
ament_target_dependencies(lqr_controller geometry_msgs nav_msgs rclcpp)

add_executable(mpc_controller src/mpc_controller.cpp)
target_compile_features(mpc_controller PUBLIC cxx_std_17)
ament_target_dependencies(mpc_controller geometry_msgs nav_msgs rclcpp std_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mpc_controller PRIVATE -O3)
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(nonlinear_mpc_test test/nonlinear_mpc_test.cpp)
  target_compile_features(nonlinear_mpc_test PRIVATE cxx_std_17)
  target_include_directories(nonlinear_mpc_test PRIVATE src)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(nonlinear_mpc_test PRIVATE -O3)
  endif()
  add_test(NAME nonlinear_mpc_test COMMAND nonlinear_mpc_test)
endif()

install(TARGETS gazebo_actuator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS quadcopter_controller lqr_controller mpc_controller motor_allocator
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/trajectory_node.py scripts/control_evaluator.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch models worlds config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
