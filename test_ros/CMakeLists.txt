cmake_minimum_required(VERSION 3.5)
project(test_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(/opt/ros/foxy/include/)

add_executable(talker src/test_publish.cc)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/test_subscrib.cc)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(polygon_publish src/test_publish_polygon.cc)
ament_target_dependencies(polygon_publish rclcpp std_msgs geometry_msgs) 

add_executable(map_tf src/test_publish_static_tf.cc)
ament_target_dependencies(
  map_tf
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

add_executable(vehicle_tf src/test_publish_dynamic_tf.cc)
ament_target_dependencies(
  vehicle_tf
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
   std_msgs
)

install(TARGETS
  talker
  listener
  polygon_publish
  map_tf
  vehicle_tf
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()


