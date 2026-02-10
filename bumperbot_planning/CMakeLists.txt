cmake_minimum_required(VERSION 3.8)
project(bumperbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# C++ specific build commands (now uncommented to fix clangd detection)
add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp nav_msgs geometry_msgs tf2_ros)
target_include_directories(dijkstra_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner rclcpp nav_msgs geometry_msgs tf2_ros)
target_include_directories(a_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS dijkstra_planner a_star_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

install(PROGRAMS
  ${PROJECT_NAME}/dijkstra_planner.py
  ${PROJECT_NAME}/dijkstra_planner_optimized.py
  ${PROJECT_NAME}/bfs_planner.py
  ${PROJECT_NAME}/a_star_planner.py
  ${PROJECT_NAME}/a_star_planner_optimized.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${PROJECT_NAME}/graph_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
