cmake_minimum_required(VERSION 3.8)
project(rosbot_xl_cpp_examples)

# Set C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(demo_messaging_node src/demo_messaging_node.cpp)
ament_target_dependencies(demo_messaging_node rclcpp sensor_msgs std_msgs std_srvs)

install(TARGETS
  demo_messaging_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

