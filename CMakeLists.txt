cmake_minimum_required(VERSION 3.16)
project(multi_robot_exploration CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/cpp/include
    ${CMAKE_SOURCE_DIR}/cpp/third_party
)

# Toàn bộ logic core nằm trong header-only (.hpp), nên simulator chỉ cần biên dịch main.cpp.
add_executable(simulator cpp/app/main.cpp)
target_link_libraries(simulator PRIVATE Threads::Threads)

# ---- Tests (tuỳ chọn, bật bằng -DBUILD_TESTS=ON) ----
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_executable(test_astar tests/test_astar.cpp)
    target_link_libraries(test_astar PRIVATE Threads::Threads)
    add_test(NAME test_astar COMMAND test_astar)

    add_executable(test_frontier tests/test_frontier.cpp)
    target_link_libraries(test_frontier PRIVATE Threads::Threads)
    add_test(NAME test_frontier COMMAND test_frontier)

    add_executable(test_collision tests/test_collision.cpp)
    target_link_libraries(test_collision PRIVATE Threads::Threads)
    add_test(NAME test_collision COMMAND test_collision)

    add_executable(test_shared_map tests/test_shared_map.cpp)
    target_link_libraries(test_shared_map PRIVATE Threads::Threads)
    add_test(NAME test_shared_map COMMAND test_shared_map)
endif()
