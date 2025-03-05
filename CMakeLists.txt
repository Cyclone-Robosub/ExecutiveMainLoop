cmake_minimum_required(VERSION 3.5)
project(executive_main_loop)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


add_executable(ExecutiveLoop 
        main.cpp)
  

# GTest start
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/d144031940543e15423a25ae5a8a74141044862f.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

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
