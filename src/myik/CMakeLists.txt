cmake_minimum_required(VERSION 3.8)
project(myik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(my_ik_node src/my_ik_node.cpp)

# 使用 ament_target_dependencies 添加 ROS 依赖（包含路径、库等）
ament_target_dependencies(my_ik_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# 单独链接 Qt 库
target_link_libraries(my_ik_node
  Qt5::Widgets
  Qt5::Core
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