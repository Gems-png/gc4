cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(complete_flow src/complete_flow.cpp)
ament_target_dependencies(complete_flow rclcpp)

add_executable(grab1 src/grab1.cpp)
ament_target_dependencies(grab1 rclcpp)

add_executable(grab2 src/grab2.cpp)
ament_target_dependencies(grab2 rclcpp)

add_executable(put src/put.cpp)
ament_target_dependencies(put rclcpp)

install(TARGETS
  complete_flow
  grab1
  grab2
  put
  DESTINATION lib/${PROJECT_NAME}
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
