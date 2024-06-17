cmake_minimum_required(VERSION 3.5)
project(cam_sub)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

link_directories(src)
# Add executable and implementation files
add_executable(image_subscriber 
  src/image_subscriber.cpp 
  src/ximage_processing.c
  src/ximage_processing_sinit.c
  src/ximage_processing_linux.c
  src/ximage_processing_hw.h # Add this implementation file
)

# Link against dependencies
ament_target_dependencies(image_subscriber rclcpp std_msgs sensor_msgs cv_bridge)

# Include directories for headers
target_include_directories(image_subscriber PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Install the target
install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
