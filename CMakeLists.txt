cmake_minimum_required(VERSION 3.23)
project(ExecutiveMainLoop)

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


find_package(Threads REQUIRED)
target_link_libraries(ExecutiveLoop Threads::Threads)
add_subdirectory(yaml-cpp)
target_link_libraries(ExecutiveLoop yaml-cpp)
#target_link_libraries(run_tests yaml-cpp)


#Gtest end



