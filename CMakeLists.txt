cmake_minimum_required(VERSION 3.8)
project(bezier_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(bezier src/bezier.cpp src/utils/bezier_math.cpp)
ament_target_dependencies(bezier rclcpp geometry_msgs turtlesim)

target_include_directories(bezier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bezier PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS bezier
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Add your tests here
endif()

ament_package()