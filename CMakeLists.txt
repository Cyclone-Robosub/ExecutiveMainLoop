cmake_minimum_required(VERSION 3.20)
project(executive_main_loop)

# Add dependency locations
add_subdirectory(lib/yaml-cpp)
add_subdirectory(lib/Executive_Propulsion)

# Find required packages
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXPORT_COMPILE_COMMANDS True)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DIDINS1.msg"
  DEPENDENCIES std_msgs builtin_interfaces
  )
# Main executable
add_executable(ExecutiveExecutable
  ExecutiveLoop.cpp
  SetupConfig/SetupRobot.cpp
)
install(TARGETS ExecutiveExecutable DESTINATION lib/${PROJECT_NAME})

# Dependencies for main executable
ament_target_dependencies(ExecutiveExecutable
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  sensor_msgs
  std_srvs
)
target_link_libraries(ExecutiveExecutable
  wiringPi
  PropulsionFunctions
  yaml-cpp
  executive_main_loop__rosidl_typesupport_cpp
)

# Include directories
target_include_directories(ExecutiveExecutable PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/SetupConfig
  ${CMAKE_CURRENT_SOURCE_DIR}/Manipulation
  ${CMAKE_CURRENT_SOURCE_DIR}/PropulsionFunctions
)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion
  ${CMAKE_CURRENT_SOURCE_DIR}/build/executive_main_loop/rosidl_generator_cpp/
  ${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/build/inertial_sense_ros2/rosidl_generator_cpp/
)
find_library(wiringPi_LIB wiringPi)


target_link_libraries(ExecutiveExecutable ${wiringPi_LIB})
ament_export_include_directories(ros2_ws/src/inertial-sense-sdk/ROS/ros2/shared/include)
ament_export_libraries(inertial_sense_ros2)
ament_export_dependencies(rclcpp std_msgs)
ament_package()

# Testing configuration
if(BUILD_TESTING)
  # Add test executable with test directory files
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(run_Executive_tests test/test_executive_loop.cpp
    test/test_main.cpp
    test/test_nodes.hpp
    test/test_sensor_data.cpp)


  # Testing definitions and includes
  target_compile_definitions(run_Executive_tests PRIVATE TESTING_EXCLUDE_MAIN)
  target_include_directories(run_Executive_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/gtest/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
target_link_libraries(ExecutiveExecutable
  wiringPi
  PropulsionFunctions
  yaml-cpp
)
  # Dependencies and linking
  ament_target_dependencies(run_Executive_tests
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
  )
  target_link_libraries(run_Executive_tests
    gtest
    gtest_main
    pthread

    PropulsionFunctions
    yaml-cpp
  )
  # Install and discover tests
  install(TARGETS run_Executive_tests DESTINATION lib/${PROJECT_NAME})
endif()

# Final configuration
enable_testing()
find_package(Threads REQUIRED)