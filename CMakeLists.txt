cmake_minimum_required(VERSION 3.20)
project(executive_main_loop)

add_subdirectory(lib/gtest)
add_subdirectory(lib/yaml-cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(ExecutiveLoop main.cpp)
  
# GTest start
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

target_link_libraries(ExecutiveLoop gtest gtest_main)

enable_testing()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
ament_target_dependencies(ExecutiveLoop rclcpp std_msgs)
install(TARGETS
  ExecutiveLoop
  DESTINATION lib/${PROJECT_NAME})

ament_package()

find_package(Threads REQUIRED)
target_link_libraries(ExecutiveLoop Threads::Threads)


#target_link_libraries(run_tests yaml-cpp)


#Gtest end




# GTest end
