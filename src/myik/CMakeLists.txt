cmake_minimum_required(VERSION 3.8)
project(myik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 可视化搬到 RViz，不再依赖 Qt（车上跑也不用装 Qt 了）
add_executable(my_ik_node src/my_ik_node.cpp)

ament_target_dependencies(my_ik_node
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

target_compile_features(my_ik_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS my_ik_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()