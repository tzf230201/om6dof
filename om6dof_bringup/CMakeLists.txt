cmake_minimum_required(VERSION 3.8)
project(om6dof_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_torque_off_diagnostic
    test/test_torque_off_diagnostic.py
  )
endif()

ament_package()
