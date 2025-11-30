cmake_minimum_required(VERSION 3.8)
project(robert)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(solver_node src/robert.cpp)
target_include_directories(solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(solver_node PUBLIC c_std_17)
ament_target_dependencies(solver_node
  rclcpp
  cg_interfaces
)

install(TARGETS solver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
