# cmake_minimum_required(VERSION 3.5)
# project(ur_pick_and_place)

# # -------------------------------
# # Dependencias C++
# # -------------------------------
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# # Optional but often present:
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning REQUIRED)

# # Nodo en C++
# add_executable(pick_place_node src/pick_place_node.cpp)
# add_executable(go_home_node src/go_home_node.cpp)

# ament_target_dependencies(pick_place_node rclcpp geometry_msgs moveit_ros_planning_interface moveit_core moveit_ros_planning)

# install(TARGETS pick_place_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# # GoHomeNode
# add_executable(go_home_node src/go_home_node.cpp)
# target_compile_features(go_home_node PUBLIC cxx_std_17)
# ament_target_dependencies(go_home_node
#   rclcpp
#   moveit_ros_planning_interface
#   moveit_core
#   moveit_ros_planning
# )
# install(TARGETS go_home_node DESTINATION lib/${PROJECT_NAME})

# # -------------------------------
# # Soporte para nodos Python (futuro)
# # -------------------------------
# find_package(ament_cmake_python REQUIRED)

# ament_python_install_package(${PROJECT_NAME})

# # Ejemplo de instalación de un nodo Python (cuando lo tengas):
# # install(PROGRAMS
# #   ur_pick_and_place/pick_place.py
# #   DESTINATION lib/${PROJECT_NAME}
# # )

# ament_package()


cmake_minimum_required(VERSION 3.8)
project(ur_pick_and_place)

# -------------------------------
# Dependencies
# -------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # for std_msgs/Empty in go_home_node
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

# -------------------------------
# C++ nodes
# -------------------------------

# pick_and_place Node
add_executable(pick_and_place src/pick_and_place.cpp)
target_compile_features(pick_and_place PUBLIC cxx_std_17)
ament_target_dependencies(pick_and_place
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
)

# GoHome Node
add_executable(go_home_node src/go_home_node.cpp)  # (single definition)
target_compile_features(go_home_node PUBLIC cxx_std_17)
ament_target_dependencies(go_home_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
)


install(TARGETS
  pick_and_place
  go_home_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# (Optional) Python support
# -------------------------------
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# install(PROGRAMS
#   ur_pick_and_place/pick_place.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()