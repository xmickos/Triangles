cmake_minimum_required(VERSION 3.11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project("triangles")
project("trianglestest")

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
set(SRCS include/algorithm.cpp include/triangles.hpp include/bvh.hpp include/generate_tests.cpp)

add_library(
    triangleslib
    STATIC ${SRCS}
)

add_executable(triangles
    ${SRCS} triangles.cpp
)
add_executable(trianglestest
    ${SRCS} trianglestest.cpp
)

target_link_libraries(triangles
    PRIVATE triangleslib
)
target_link_libraries(trianglestest
    PRIVATE
        GTest::gtest_main
        triangleslib
)

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")


target_compile_features(triangleslib
    PUBLIC cxx_std_20
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(triangles
        PRIVATE
            -g
            -O0
    )
    target_compile_options(triangleslib
        PUBLIC
            -g
            -O0
    )
    target_compile_options(trianglestest
        PUBLIC
            -g
            -O0
    )

endif()

enable_testing()
include(GoogleTest)
gtest_discover_tests(trianglestest)
