cmake_minimum_required(VERSION 3.20)
project(waypoint_decision)

# Option to mock RPi (Pico replacement)
option(MOCK_RPI "Use mock RPi functions (no Pico)" OFF)
set(MOCK_RPI ${MOCK_RPI} CACHE BOOL "Use mock RPi functions (no Pico)" FORCE)
add_compile_definitions($<$<BOOL:MOCK_RPI>:MOCK_RPI>)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# Compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)

# Add external libraries
add_subdirectory(lib/Executive_Propulsion)

# Main executable
add_executable(ExecutiveExecutable
  WaypointDecision/mainWaypoint.cpp
  WaypointDecision/Task.hpp
  WaypointDecision/Task.cpp
  WaypointDecision/WaypointExecutive.hpp
  WaypointDecision/WaypointExecutive.cpp
  WaypointDecision/JSON_Parser/MissionAnalyser.hpp
)

target_include_directories(ExecutiveExecutable PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}/SetupConfig
  ${CMAKE_CURRENT_SOURCE_DIR}/Manipulation
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion/src
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Json
)

target_link_libraries(ExecutiveExecutable
  PropulsionFunctions
)

ament_target_dependencies(ExecutiveExecutable
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  std_srvs
)

install(TARGETS ExecutiveExecutable DESTINATION lib/${PROJECT_NAME})

ament_package()
