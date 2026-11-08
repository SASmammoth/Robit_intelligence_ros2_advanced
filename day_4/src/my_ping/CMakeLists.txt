cmake_minimum_required(VERSION 3.8)
project(my_ping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pingpong_lab_interfaces REQUIRED)

include_directories(include)

add_executable(my_ping_node src/my_ping_node.cpp)
target_link_libraries(my_ping_node rclcpp::rclcpp ${pingpong_lab_interfaces_TARGETS})

install(TARGETS my_ping_node DESTINATION lib/${PROJECT_NAME})

ament_package()
