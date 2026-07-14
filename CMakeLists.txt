cmake_minimum_required(VERSION 3.8)
project(sura_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sura_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/diagnostics_monitor.cpp
  src/limits_safety_nodes.cpp
  src/register_nodes.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE BT_PLUGIN_EXPORT
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  diagnostic_msgs
  behaviortree_cpp_v3
  sura_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  diagnostic_msgs
  behaviortree_cpp_v3
  sura_msgs
)

ament_package()
