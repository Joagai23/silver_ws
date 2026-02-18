cmake_minimum_required(VERSION 3.5)
project(silver)
cmake_policy(SET CMP0009 NEW)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

#add_executable(path_following scripts/path_following.py)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

#install python executables
install(
  DIRECTORY config description launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/old_implementation/forward_position_publisher.py  scripts/old_implementation/position_trajectory_publisher.py
  scripts/old_implementation/follow_joint_position_trajectory_action_client.py scripts/old_implementation/forward_position_publisher_silver.py
  scripts/old_implementation/silver_follow_joint_position_trajectory_action.py scripts/old_implementation/trajectory_publisher.py
  scripts/omnidirectional/cmd_vel_control_rotation.py scripts/omnidirectional/robot_def.py scripts/omnidirectional/robot.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
