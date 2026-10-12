cmake_minimum_required(VERSION 3.8)
# Modified for OM6DOF ROS 2/ament integration; see LICENSE and NOTICE.
project(om6dof_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(FILES LICENSE NOTICE README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config docs launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(PROGRAMS
  scripts/com_markers.py
  scripts/static_joint_states.py
  scripts/cog_vectors
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(test_name feedback vectors jacobian rviz_launch view_dynamic_launch)
    ament_add_pytest_test(${test_name} test/test_${test_name}.py
      TIMEOUT 120
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
