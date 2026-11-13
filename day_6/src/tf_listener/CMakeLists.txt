cmake_minimum_required(VERSION 3.8)
project(tf_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ---- 기존 노드 ----
add_executable(calc_my_IK_node src/calc_my_IK_node.cpp)
target_include_directories(calc_my_IK_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(calc_my_IK_node PUBLIC cxx_std_17)
ament_target_dependencies(calc_my_IK_node rclcpp std_msgs tf2 tf2_ros geometry_msgs)
install(TARGETS calc_my_IK_node DESTINATION lib/${PROJECT_NAME})

# ---- 새 노드 ----
foreach(node ik_teleop_node ik_demo_node)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(${node} PUBLIC cxx_std_17)
  ament_target_dependencies(${node} rclcpp tf2 tf2_ros geometry_msgs sensor_msgs)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
