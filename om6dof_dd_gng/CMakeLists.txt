cmake_minimum_required(VERSION 3.16)
project(om6dof_dd_gng LANGUAGES C CXX)

option(BUILD_DDGNG_VISUALIZER
  "Build the legacy OpenGL/ODE DepthSensor_Buggy visualizer" OFF)

if(BUILD_DDGNG_VISUALIZER)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(ODE_PC REQUIRED ode)
# MSYS2's ode.pc lies about the library name (says -lode, ships libode_double),
# so resolve the real .dll.a ourselves and only borrow include dirs from pkg-config.
find_library(ODE_LIBRARY NAMES ode_double ode REQUIRED HINTS ${ODE_PC_LIBRARY_DIRS})
add_library(ODE::ODE INTERFACE IMPORTED)
target_include_directories(ODE::ODE INTERFACE ${ODE_PC_INCLUDE_DIRS})
target_link_libraries(ODE::ODE INTERFACE ${ODE_LIBRARY})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Vendored drawstuff (ODE 0.16.6) built as a static lib.
set(DS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/drawstuff)
set(DS_SOURCES ${DS_DIR}/src/drawstuff.cpp)
if(WIN32)
  list(APPEND DS_SOURCES ${DS_DIR}/src/windows.cpp)
elseif(APPLE)
  list(APPEND DS_SOURCES ${DS_DIR}/src/osx.cpp)
else()
  list(APPEND DS_SOURCES ${DS_DIR}/src/x11.cpp)
endif()

add_library(drawstuff STATIC ${DS_SOURCES})
target_include_directories(drawstuff
  PUBLIC  ${DS_DIR}/include
  PRIVATE ${DS_DIR}/src)
target_compile_definitions(drawstuff PUBLIC dDOUBLE)
target_link_libraries(drawstuff
  PUBLIC ODE::ODE OpenGL::GL OpenGL::GLU)
if(WIN32)
  target_link_libraries(drawstuff PUBLIC gdi32 winmm user32 comdlg32)
elseif(NOT APPLE)
  find_package(X11 REQUIRED)
  target_link_libraries(drawstuff PUBLIC X11::X11)
endif()

# Main application
set(APP_SOURCES
  DepthSensor_Buggy/main.cpp
  DepthSensor_Buggy/environment.cpp
  DepthSensor_Buggy/extDisplay.cpp
  DepthSensor_Buggy/gng.cpp
  DepthSensor_Buggy/malloc.cpp
  DepthSensor_Buggy/mesh.cpp
  DepthSensor_Buggy/projection.cpp
  DepthSensor_Buggy/rnd.cpp
  DepthSensor_Buggy/surfMatching.cpp)

# Drawstuff's Win32 resources (menu/accelerators/about-dialog) must live in the
# final PE — windres-produced resource objects don't survive being archived
# inside a static library, so the LoadAccelerators call at startup would fail.
if(WIN32)
  list(APPEND APP_SOURCES ${DS_DIR}/src/resources.rc)
endif()

add_executable(DepthSensor_Buggy ${APP_SOURCES})
if(WIN32)
  target_include_directories(DepthSensor_Buggy PRIVATE ${DS_DIR}/src)
endif()
target_compile_definitions(DepthSensor_Buggy PRIVATE dDOUBLE _USE_MATH_DEFINES)
target_link_libraries(DepthSensor_Buggy PRIVATE
  drawstuff ODE::ODE GLUT::GLUT OpenGL::GL OpenGL::GLU)

# Copy textures next to the executable so fn.path_to_textures = "textures" works.
add_custom_command(TARGET DepthSensor_Buggy POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${DS_DIR}/textures $<TARGET_FILE_DIR:DepthSensor_Buggy>/textures)

install(TARGETS DepthSensor_Buggy RUNTIME DESTINATION bin)
endif()

install(DIRECTORY systemd/ DESTINATION share/${PROJECT_NAME}/systemd)

# ============================================================================
# topo_gng_node: the ROS 2 (rclcpp) TopoVLA DD-GNG + robot-topology integration.
# Everything above this point is the pre-existing plain-CMake sub-build
# (legacy visualizer + realsense_ddgng's own build_om6dof); it is untouched.
# This package was a plain CMake package (identified by colcon on
# CMakeLists.txt alone, no package.xml); it now also carries a package.xml and
# becomes ament_cmake so it can depend on rclcpp etc. colcon still builds both
# halves of this one CMakeLists.txt in a single pass.
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# Provided by the ros-humble-librealsense2 apt package (sourcing
# /opt/ros/humble/setup.bash puts its CMake config on CMAKE_PREFIX_PATH); this
# is the same librealsense2.so the Python realsense_ddgng/ tools load via
# pyrealsense2, so topo_gng_node and dd_gng_yolo.py contend for the same
# camera handle exactly like any two processes opening the same device would.
find_package(realsense2 REQUIRED)

add_executable(topo_gng_node
  src/topo_gng_node.cpp
)
target_include_directories(topo_gng_node PRIVATE
  include
)
target_compile_features(topo_gng_node PRIVATE cxx_std_17)
ament_target_dependencies(topo_gng_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(topo_gng_node
  realsense2::realsense2
  ${OpenCV_LIBS}
)
target_include_directories(topo_gng_node PRIVATE ${OpenCV_INCLUDE_DIRS})

install(TARGETS topo_gng_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
