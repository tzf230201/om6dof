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
# The legacy ODE visualizer above remains optional. The native Python
# camera core below is built and installed in the same ament/colcon pass.
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Provided by the ros-humble-librealsense2 apt package (sourcing
# /opt/ros/humble/setup.bash puts its CMake config on CMAKE_PREFIX_PATH); this
# is the same librealsense2.so the Python realsense_ddgng/ tools load via
# pyrealsense2, so topo_gng_node and dd_gng_yolo.py contend for the same
# camera handle exactly like any two processes opening the same device would.
find_package(realsense2 REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include)
find_library(TENSORRT_NVINFER_LIBRARY nvinfer REQUIRED)
find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h
  HINTS /usr/local/cuda/include)
find_library(CUDA_RUNTIME_LIBRARY cudart
  HINTS /usr/local/cuda/lib64 /usr/local/cuda/targets/aarch64-linux/lib)
if(NOT TENSORRT_INCLUDE_DIR OR NOT CUDA_RUNTIME_INCLUDE_DIR OR NOT CUDA_RUNTIME_LIBRARY)
  message(FATAL_ERROR "TensorRT and CUDA runtime development files are required for topo_gng_node")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TopologyEdge.msg"
  "msg/EnvironmentNode.msg"
  "msg/EnvironmentGraph.msg"
  "msg/ReachabilityNode.msg"
  "msg/ReachabilityGraph.msg"
  "msg/ReachabilityPlan.msg"
  "msg/ReachabilityQuery.msg"
  "srv/ValidateReachabilityScene.srv"
  "srv/ValidateGraspExecution.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs trajectory_msgs nav_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(topo_gng_node
  src/topo_gng_node.cpp
)
target_include_directories(topo_gng_node PRIVATE
  include
)

add_executable(yolox_viewer_node
  src/yolox_viewer_node.cpp
)
target_include_directories(yolox_viewer_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_RUNTIME_INCLUDE_DIR}
)
target_compile_features(yolox_viewer_node PRIVATE cxx_std_17)
ament_target_dependencies(yolox_viewer_node rclcpp sensor_msgs std_msgs)
target_link_libraries(yolox_viewer_node
  ${OpenCV_LIBS}
  ${TENSORRT_NVINFER_LIBRARY}
  ${CUDA_RUNTIME_LIBRARY}
)

# Standalone YOLO + 3D box + DD-GNG node: owns the camera itself (no
# dependency on topo_gng_node's RGB stream), runs YOLOX synchronously once
# per captured frame, and feeds a DynamicDensityGrowingNeuralGas whose
# attention is pulled toward each frame's YOLO/3D-box detections -- no TF, no
# MoveIt, no cross-frame re-projection (everything happens in the same
# camera-frame instant).
add_executable(yolo_3d_box_node
  src/yolo_3d_box_node.cpp
)
target_include_directories(yolo_3d_box_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_RUNTIME_INCLUDE_DIR}
)
target_compile_features(yolo_3d_box_node PRIVATE cxx_std_17)
ament_target_dependencies(yolo_3d_box_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(yolo_3d_box_node
  realsense2::realsense2
  ${OpenCV_LIBS}
  ${TENSORRT_NVINFER_LIBRARY}
  ${CUDA_RUNTIME_LIBRARY}
  ${cpp_typesupport_target}
)
target_compile_features(topo_gng_node PRIVATE cxx_std_17)
ament_target_dependencies(topo_gng_node
  rclcpp
  rcl_interfaces
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
  ${TENSORRT_NVINFER_LIBRARY}
  ${CUDA_RUNTIME_LIBRARY}
  ${cpp_typesupport_target}
)
target_include_directories(topo_gng_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_RUNTIME_INCLUDE_DIR}
)

add_executable(reachability_graph_node
  src/reachability_graph_node.cpp
)
target_include_directories(reachability_graph_node PRIVATE include)
target_compile_features(reachability_graph_node PRIVATE cxx_std_17)
ament_target_dependencies(reachability_graph_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_msgs
  shape_msgs
)
target_link_libraries(reachability_graph_node ${cpp_typesupport_target})

install(TARGETS topo_gng_node yolox_viewer_node yolo_3d_box_node reachability_graph_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
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
install(PROGRAMS
  scripts/semantic_target_gui.py
  scripts/reachability_benchmark.py
  scripts/reachability_multiscene_benchmark.py
  scripts/generate_reachability_scene_catalog.py
  scripts/analyze_reachability_multiscene.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the headless-capable Python camera/web path with its native core,
# so launching it no longer depends on a separately built source-tree .so.
add_subdirectory(realsense_ddgng)
install(TARGETS ddgng LIBRARY DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  realsense_ddgng/dd_gng_yolo.py
  realsense_ddgng/ddgng_realsense.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES realsense_ddgng/camera_input.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_dynamic_density_gng test/test_dynamic_density_gng.cpp)
  target_include_directories(test_dynamic_density_gng PRIVATE include)
  target_compile_features(test_dynamic_density_gng PRIVATE cxx_std_17)
  ament_add_gtest(test_semantic_density_attention test/test_semantic_density_attention.cpp)
  target_include_directories(test_semantic_density_attention PRIVATE include)
  target_compile_features(test_semantic_density_attention PRIVATE cxx_std_17)
  ament_add_gtest(test_semantic_cluster_propagation test/test_semantic_cluster_propagation.cpp)
  target_include_directories(test_semantic_cluster_propagation PRIVATE include)
  target_compile_features(test_semantic_cluster_propagation PRIVATE cxx_std_17)
  ament_add_gtest(test_camera_profile test/test_camera_profile.cpp)
  target_include_directories(test_camera_profile PRIVATE include)
  target_compile_features(test_camera_profile PRIVATE cxx_std_17)
  ament_add_pytest_test(test_v2_launch test/test_v2_launch.py TIMEOUT 30)
  ament_add_pytest_test(test_web_launch test/test_web_launch.py TIMEOUT 30)
  ament_add_pytest_test(test_camera_input realsense_ddgng/test/test_camera_input.py TIMEOUT 30)
  ament_add_pytest_test(test_dd_gng_yolo realsense_ddgng/test/test_dd_gng_yolo.py TIMEOUT 30)
  ament_add_gtest(test_workspace_samples test/test_workspace_samples.cpp)
  target_include_directories(test_workspace_samples PRIVATE include)
  ament_add_gtest(test_semantic_target_selection test/test_semantic_target_selection.cpp)
  target_include_directories(test_semantic_target_selection PRIVATE include)
  target_compile_features(test_semantic_target_selection PRIVATE cxx_std_17)
  ament_add_gtest(test_final_grasp test/test_final_grasp.cpp)
  target_include_directories(test_final_grasp PRIVATE include)
  target_compile_features(test_final_grasp PRIVATE cxx_std_17)
  ament_target_dependencies(test_final_grasp moveit_core)
  ament_add_gtest(test_final_grasp_robot test/test_final_grasp_robot.cpp)
  target_include_directories(test_final_grasp_robot PRIVATE include)
  target_compile_features(test_final_grasp_robot PRIVATE cxx_std_17)
  ament_target_dependencies(test_final_grasp_robot moveit_core moveit_ros_planning)
  ament_add_gtest(test_grasp_collision_scene test/test_grasp_collision_scene.cpp)
  target_include_directories(test_grasp_collision_scene PRIVATE include)
  target_compile_features(test_grasp_collision_scene PRIVATE cxx_std_17)
  ament_target_dependencies(test_grasp_collision_scene
    moveit_core moveit_ros_planning moveit_msgs shape_msgs geometry_msgs)
  target_link_libraries(test_grasp_collision_scene ${cpp_typesupport_target})
  ament_add_pytest_test(test_experiment_launch test/test_experiment_launch.py TIMEOUT 30)

  ament_add_gtest(test_reachability_graph
    test/test_reachability_graph.cpp
  )
  target_include_directories(test_reachability_graph PRIVATE include)
  target_compile_features(test_reachability_graph PRIVATE cxx_std_17)
  ament_add_pytest_test(test_reachability_benchmark
    test/test_reachability_benchmark.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_reachability_multiscene_benchmark
    test/test_reachability_multiscene_benchmark.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_generate_reachability_scene_catalog
    test/test_generate_reachability_scene_catalog.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_analyze_reachability_multiscene
    test/test_analyze_reachability_multiscene.py
    TIMEOUT 60
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
