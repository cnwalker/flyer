cmake_minimum_required(VERSION 3.29)
set(CMAKE_CXX_STANDARD 17)
project(FlyerSimulator)
Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0 # or a later release
)

FetchContent_MakeAvailable(Catch2)

add_executable(tests src/tests.cpp src/telem.cpp src/telem.h)
target_link_libraries(
    tests
    PRIVATE
)

target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
include(Catch)
catch_discover_tests(tests)