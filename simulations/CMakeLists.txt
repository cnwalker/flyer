cmake_minimum_required(VERSION 3.29)
set(CMAKE_CXX_STANDARD 14)
project(FlyerSimulator)

Include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

add_executable(
    simulator
    src/simulator_bin.cpp
    src/simulation.cpp
    src/simulation.h
    ../telemetry/telem.cpp
    ../telemetry/telem.h
)

target_link_libraries(
    simulator
    PRIVATE
    yaml-cpp  # This is the "target" exposed by 
              # yaml-cpp's CMakeLists.txt file
)


FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0 # or a later release
)

FetchContent_MakeAvailable(Catch2)

add_executable(tests src/tests.cpp src/simulation.h src/simulation.cpp)
target_link_libraries(
    tests
    PRIVATE
    yaml-cpp  # This is the "target" exposed by
              # yaml-cpp's CMakeLists.txt file
)

target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
include(Catch)
catch_discover_tests(tests)