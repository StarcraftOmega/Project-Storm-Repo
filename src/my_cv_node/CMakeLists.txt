# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.8)

# Declare the package name
project(my_image_package)

# Find necessary ROS 2 and build system packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Find the system-installed OpenCV package
# cv_bridge depends on this, but it's good practice to find it explicitly
find_package(OpenCV REQUIRED)

# Define the executable
# This assumes your C++ file is named "image_processor_node.cpp"
# and is located in the "src/" directory.
add_executable(image_processor_node src/image_processor_node.cpp)

# Link the ROS 2 dependencies
ament_target_dependencies(image_processor_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Link the non-ROS OpenCV libraries
# This is crucial for cv::imshow, cv::Mat, etc.
target_link_libraries(image_processor_node ${OpenCV_LIBRARIES})

# Install the executable
# This tells ament where to put the compiled program so "ros2 run" can find it
install(TARGETS
  image_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Register the package with the ament build system
ament_package()