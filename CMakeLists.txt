cmake_minimum_required(VERSION 3.20)
project(executive_main_loop)

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
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# Compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)

# Add external libraries
add_subdirectory(lib/Executive_Propulsion)

# ExecutivePropulsionLib
add_library(ExecutivePropulsionLib
  lib/Executive_Propulsion/lib/Command_Interpreter.cpp
  lib/Executive_Propulsion/lib/Wiring.cpp
)

# Main executable
add_executable(ExecutiveExecutable
  src/main.cpp
  src/ExecutiveLoop.cpp
  src/ExecutiveLoop.hpp
  src/SensorDataConfig.cpp
  src/SensorDataConfig.hpp
  SetupConfig/SetupRobot.cpp
)

target_include_directories(ExecutiveExecutable PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}/SetupConfig
  ${CMAKE_CURRENT_SOURCE_DIR}/Manipulation
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion/lib
)

target_link_libraries(ExecutiveExecutable
  yaml-cpp
  ExecutivePropulsionLib
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