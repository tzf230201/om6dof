cmake_minimum_required(VERSION 3.16)
project(om6dof_f_gng VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(f_gng_core INTERFACE)
target_include_directories(f_gng_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/om6dof_f_gng/core>)

add_executable(f_gng_node src/f_gng_node.cpp)
target_link_libraries(f_gng_node f_gng_core)
ament_target_dependencies(f_gng_node rclcpp rcl_interfaces tf2 tf2_ros
  geometry_msgs sensor_msgs std_msgs visualization_msgs diagnostic_msgs std_srvs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(f_gng_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
install(TARGETS f_gng_node DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/realsense_depth_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz docs paper DESTINATION share/${PROJECT_NAME})
install(FILES README.md THIRD_PARTY_NOTICE.md CORE_PROVENANCE.json BASELINE_PROVENANCE.json
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  foreach(component focus mapper)
    add_executable(f_gng_${component}_test tests/${component}_selftest.cpp)
    target_link_libraries(f_gng_${component}_test PRIVATE f_gng_core)
    add_test(NAME f_gng_${component} COMMAND f_gng_${component}_test)
  endforeach()
  add_executable(gng_comparison_test tests/comparison_mapper_selftest.cpp)
  target_link_libraries(gng_comparison_test PRIVATE f_gng_core)
  add_test(NAME gng_comparison COMMAND gng_comparison_test)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(realsense_depth tests/test_realsense_depth.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(ros_pipeline tests/test_ros_pipeline.py
    TIMEOUT 90 ENV F_GNG_NODE=$<TARGET_FILE:f_gng_node>)
  ament_add_pytest_test(baseline_launch tests/test_baseline_launch.py)
endif()

ament_package()
