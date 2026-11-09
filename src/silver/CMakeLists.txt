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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install directories
install(
  DIRECTORY config description launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS
  scripts/locomotion_manager.py
  scripts/dynamixel/reboot_servos.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
