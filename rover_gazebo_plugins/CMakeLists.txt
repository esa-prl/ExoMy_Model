cmake_minimum_required(VERSION 3.5)
project(rover_gazebo_plugins)

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
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
# custom packages
find_package(exomy_sim_msgs REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${exomy_sim_msgs_INCLUDE_DIRS})

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(rover_gazebo_joint_plugin SHARED
  src/rover_gazebo_joint_plugin.cpp)

ament_target_dependencies(rover_gazebo_joint_plugin
  "gazebo_dev"
  "gazebo_ros"
  "exomy_sim_msgs"
  "rclcpp"
  "ament_index_cpp")
ament_export_libraries(rover_gazebo_joint_plugin)

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

ament_package()

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS
  rover_gazebo_joint_plugin
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)