cmake_minimum_required(VERSION 3.8)
project(rt2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(rt2_interfaces REQUIRED)

add_executable(node2_controller src/node2_controller.cpp)
ament_target_dependencies(node2_controller rclcpp geometry_msgs sensor_msgs rt2_interfaces)

install(
  TARGETS node2_controller 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install Python scripts
install(PROGRAMS 
  scripts/node1_ui.py    
  DESTINATION lib/${PROJECT_NAME} 
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()