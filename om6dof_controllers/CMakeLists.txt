cmake_minimum_required(VERSION 3.16)
project(om6dof_controllers LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  kdl_parser
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rcl_interfaces
  realtime_tools
  sensor_msgs
  std_msgs
  trajectory_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/controller_utils.cpp
  src/gravity_compensation_controller.cpp
  src/gravity_model.cpp
  src/leader_arm_controller.cpp
  src/spring_actuator_controller.cpp
  src/trajectory_controller.cpp
  src/trajectory_sampler.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "OM6DOF_CONTROLLERS_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${orocos_kdl_LIBRARIES})

pluginlib_export_plugin_description_file(controller_interface om6dof_controllers.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY config docs launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  PROGRAMS
    scripts/controller_node
    scripts/combined_preview_node
    scripts/compare_paper_identification.py
    scripts/identification_tracking.py
    scripts/check_current_torque.py
    scripts/fit_gravity_friction.py
    scripts/multisine_identification.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(test_name control_math ik_solver target_planner combined_reference
      combined_preview_node controller_node controller_launch)
    ament_add_pytest_test(${test_name} test/test_${test_name}.py
      TIMEOUT 120
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()

  ament_add_gtest(test_trajectory_sampler test/test_trajectory_sampler.cpp)
  target_link_libraries(test_trajectory_sampler ${PROJECT_NAME})

  ament_add_gtest(test_gravity_model test/test_gravity_model.cpp)
  target_link_libraries(test_gravity_model ${PROJECT_NAME})

  ament_add_gtest(test_com_configuration test/test_com_configuration.cpp)
  target_link_libraries(test_com_configuration ${PROJECT_NAME})
  target_compile_definitions(test_com_configuration PRIVATE
    TEST_CONTROLLER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/om6dof_controllers.yaml")

  ament_add_gtest(test_controller_utils test/test_controller_utils.cpp)
  target_link_libraries(test_controller_utils ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} orocos_kdl)
ament_package()
