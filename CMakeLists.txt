cmake_minimum_required(VERSION 3.5)
project(ur_pick_and_place)

# -------------------------------
# Dependencias C++
# -------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Nodo en C++
add_executable(pick_place_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_node rclcpp geometry_msgs moveit_ros_planning_interface)

install(TARGETS pick_place_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# Soporte para nodos Python (futuro)
# -------------------------------
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Ejemplo de instalación de un nodo Python (cuando lo tengas):
# install(PROGRAMS
#   ur_pick_and_place/pick_place.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
