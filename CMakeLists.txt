cmake_minimum_required(VERSION 3.8)
project(kml_parser)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(LibKML REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

set(INCLUDE_DIRS /usr/local/include/)
set(LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)
link_directories(${LIBRARY_DIRS})

add_executable(kml_parser_node src/kml_parser_node.cpp)
ament_target_dependencies(kml_parser_node rclcpp std_msgs sensor_msgs GeographicLib LibKML nav_msgs)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

target_link_libraries(kml_parser_node ${LIBRARY_DIRS}/libkmlbase.so ${LIBRARY_DIRS}/libkmldom.so ${LIBRARY_DIRS}/libkmlengine.so ${GeographicLib_LIBRARIES})

install(TARGETS kml_parser_node DESTINATION lib/${PROJECT_NAME})

ament_package()
