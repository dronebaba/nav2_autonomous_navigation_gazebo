cmake_minimum_required(VERSION 3.8)
project(nav2_autonomous_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_common REQUIRED)

add_executable(set_initial_pose src/set_initial_pose.cpp)
ament_target_dependencies(set_initial_pose rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(set_nav_goal src/set_nav_goal.cpp)
ament_target_dependencies(set_nav_goal rclcpp tf2 nav2_msgs visualization_msgs tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(set_wp_client src/set_wp_client.cpp)
ament_target_dependencies(set_wp_client rclcpp rcl_action tf2 nav2_msgs visualization_msgs tf2_ros tf2_geometry_msgs geometry_msgs)

install(TARGETS
  set_initial_pose
  set_nav_goal
  set_wp_client
  DESTINATION lib/${PROJECT_NAME})

# Install Directories
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description)

install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY maps/ 
  DESTINATION share/${PROJECT_NAME}/maps)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
