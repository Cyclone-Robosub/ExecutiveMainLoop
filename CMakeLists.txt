cmake_minimum_required(VERSION 3.28)
project(ExecutiveMainLoop)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options("-DMOCK_RPI")



add_executable(ExecuteLoop)
  #"lib/LQR_Control.h" "lib/Physics.h" "testing/Physics_Testing.cpp")

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



add_subdirectory(yaml-cpp)
target_link_libraries(ExecuteLoop yaml-cpp)
#target_link_libraries(run_tests yaml-cpp)


#Gtest end



