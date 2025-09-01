cmake_minimum_required(VERSION 3.8)
project(ros2_eval_task)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# ---------------- C++ Shared Library ----------------
# Add the gazebo_utils_client library
add_library(gazebo_utils_client SHARED
  src/gazebo_utils_client.cpp
)

# Set include directories
target_include_directories(gazebo_utils_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(gazebo_utils_client
  rclcpp
  gazebo_msgs
  geometry_msgs
)

# ---------------- Model Utilities Library ----------------
add_library(model_utils SHARED
  src/model_utils.cpp
)

target_include_directories(model_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(model_utils
  ament_index_cpp
)

# ---------------- Demo Executables ----------------
add_executable(demo_gazebo_utils_client src/demo_gazebo_utils_client.cpp)
target_link_libraries(demo_gazebo_utils_client gazebo_utils_client)
ament_target_dependencies(demo_gazebo_utils_client
  rclcpp
  gazebo_msgs
  geometry_msgs
  ament_index_cpp
)

# add_executable(demo_model_selection src/demo_model_selection.cpp)
# target_link_libraries(demo_model_selection gazebo_utils_client model_utils)
# ament_target_dependencies(demo_model_selection
#   rclcpp
#   gazebo_msgs
#   geometry_msgs
#   ament_index_cpp
# )

# ---------------- Battery Spawner Node ----------------
add_executable(battery_spawner_node src/battery_spawner_node.cpp)
target_link_libraries(battery_spawner_node gazebo_utils_client model_utils)
ament_target_dependencies(battery_spawner_node
  rclcpp
  gazebo_msgs
  geometry_msgs
  ament_index_cpp
)


# ---------------- Camera Model Spawner Node ----------------
add_executable(camera_model_spawner_node src/camera_model_spawner_node.cpp)
target_link_libraries(camera_model_spawner_node gazebo_utils_client model_utils ${OpenCV_LIBS} ${cv_bridge_LIBRARIES})
ament_target_dependencies(camera_model_spawner_node
  rclcpp
  gazebo_msgs
  geometry_msgs
  ament_index_cpp
  sensor_msgs
  cv_bridge
)

install(TARGETS 
  demo_gazebo_utils_client
  # demo_model_selection
  battery_spawner_node
  camera_model_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Install Libraries ----------------
install(
  TARGETS 
  gazebo_utils_client
  model_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------- Install Executables ----------------
install(
  TARGETS 
  demo_gazebo_utils_client
  # demo_model_selection
  battery_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Install Headers ----------------
install(
  DIRECTORY include/
  DESTINATION include
)

# ---------------- Install Python Modules ----------------
ament_python_install_package(${PROJECT_NAME})

# ---------------- Install Launch, RViz, and Worlds ----------------
install(DIRECTORY
  launch
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models/
  DESTINATION $ENV{HOME}/.gazebo/models
)

# ---------------- Install Package.xml ----------------
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------- Export ----------------
ament_package()