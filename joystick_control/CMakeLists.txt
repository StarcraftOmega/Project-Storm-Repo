cmake_minimum_required(VERSION 3.8)
project(joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drivetrain_interfaces REQUIRED)

# Add external joystick library (assuming it's inside external/joystick)
add_subdirectory(external/joystick)

add_executable(joystick_node src/publisher_joystick.cpp)

# Link against joystick + ROS2
target_link_libraries(joystick_node
  joystick
)

target_include_directories(joystick_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/joystick>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

install(TARGETS
  joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(joystick_node
  rclcpp
  drivetrain_interfaces
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
