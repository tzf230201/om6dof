cmake_minimum_required(VERSION 3.8)
# Modified for OM6DOF ROS 2/ament integration; see LICENSE and NOTICE.
project(om6dof_description)

find_package(ament_cmake REQUIRED)

install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/com_markers.py
  scripts/static_joint_states.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
