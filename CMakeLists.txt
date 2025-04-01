cmake_minimum_required(VERSION 3.20)
project(executive_main_loop)

add_subdirectory(lib/gtest)
add_subdirectory(lib/yaml-cpp)
add_subdirectory(lib/Propulsion)
add_subdirectory(ros2_ws/src/inertial-sense-sdk/ROS/ros2)
include_directories(ros2_ws)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXPORT_COMPILE_COMMANDS True)
add_executable(ExecutiveExecutable ExecutiveLoop.cpp
      SetupConfig/SetupRobot.cpp)
# GTest start
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

target_link_libraries(ExecutiveExecutable gtest gtest_main)

#target_link_libraries(run_tests yaml-cpp::yaml-cpp)
target_include_directories(ExecutiveExecutable PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/inertial-sense-sdk/ROS/ros2/src)
enable_testing()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
ament_target_dependencies(ExecutiveExecutable rclcpp std_msgs)
install(TARGETS
  ExecutiveExecutable
  DESTINATION lib/${PROJECT_NAME})

ament_package()

find_package(Threads REQUIRED)
target_link_libraries(ExecutiveExecutable Threads::Threads)


#Gtest end




# GTest end
