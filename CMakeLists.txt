cmake_minimum_required(VERSION 3.11)
set(CMAKE_CXX_STANDARD 20)
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
set(SRCS include/algorithm.cpp include/utils.cpp)

add_library(
    triangleslib
    STATIC ${SRCS}
)

target_include_directories(
    triangleslib
    PUBLIC include
)

add_executable(triangles
    ${SRCS} triangles.cpp
)
target_link_libraries(triangles
    PRIVATE triangleslib
)

add_executable(trianglestest
    ${SRCS} trianglestest.cpp
)
target_link_libraries(trianglestest
    PRIVATE
        GTest::gtest_main
        triangleslib
)

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

if(DEFINED SPEED_BENCH)
    target_compile_definitions(triangles PRIVATE SPEED_BENCH=${SPEED_BENCH})
endif()

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
            -Wextra
            -Werror=implicit-function-declaration
            -Wmissing-include-dirs
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
