cmake_minimum_required(VERSION 3.20)
project(executive_main_loop)

# Option to mock RPi (wiringPi replacement)
option(MOCK_RPI "Use mock RPi functions (no wiringPi)" OFF)
set(MOCK_RPI ${MOCK_RPI} CACHE BOOL "Use mock RPi functions (no wiringPi)" FORCE)
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
  main.cpp
  SetupConfig/SetupRobot.cpp
)

target_include_directories(ExecutiveExecutable PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
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

# Testing
if(BUILD_TESTING)
  # Automatic inclusion of lint-related tests
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_executive_loop
    test/test_executive_loop.cpp
    main.cpp
  )

  target_include_directories(test_executive_loop PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/SetupConfig
    ${CMAKE_CURRENT_SOURCE_DIR}/Manipulation
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion/lib
  )

  target_link_libraries(test_executive_loop
    yaml-cpp
    ExecutivePropulsionLib
  )

  ament_target_dependencies(test_executive_loop
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
  )

  ament_add_gtest(test_sensor_data
    test/test_sensor_data.cpp
    main.cpp
  )

  target_include_directories(test_sensor_data PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/SetupConfig
    ${CMAKE_CURRENT_SOURCE_DIR}/Manipulation
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Executive_Propulsion/lib
  )

  target_link_libraries(test_sensor_data
    yaml-cpp
    ExecutivePropulsionLib
  )

  ament_target_dependencies(test_sensor_data
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
  )
endif()

install(TARGETS ExecutiveExecutable DESTINATION lib/${PROJECT_NAME})

ament_package()
