cmake_minimum_required(VERSION 3.20)
project(executive_main_loop)

add_subdirectory(lib/gtest)
add_subdirectory(lib/yaml-cpp)
add_subdirectory(lib/Executive_Propulsion/pi)


find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXPORT_COMPILE_COMMANDS True)

# Main executable
add_executable(ExecutiveExecutable ExecutiveLoop.cpp)
install(TARGETS
  ExecutiveExecutable
  DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies(ExecutiveExecutable
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  sensor_msgs
  std_srvs)
target_link_libraries(ExecutiveExecutable PropulsionFunctions)

target_include_directories(ExecutiveExecutable PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/PropulsionFunctions"
                          )
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion
   
)


# Testing configuration
if(BUILD_TESTING)
  # Add test executable
  add_executable(run_tests
    test_executive_loop.cpp
    test_sensors_data.cpp
    test_node_interaction.cpp
    test_main.cpp
  )
  
  # Testing definitions and includes
  target_compile_definitions(run_tests PRIVATE TESTING_EXCLUDE_MAIN)
  target_include_directories(run_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/gtest/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  
  # Dependencies and linking
  ament_target_dependencies(run_tests
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
  )
  target_link_libraries(run_tests
    gtest
    gtest_main
    PropulsionFunctions
  )
  
  # Install and discover tests
  install(TARGETS run_tests
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_add_gtest(run_tests)
endif()

enable_testing()
find_package(Threads REQUIRED)
ament_package()