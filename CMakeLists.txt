cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 20)
set(LIBIGL_WITH_CGAL ON)
set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
set(CMAKE_PREFIX_PATH "D:\\OwnLibrary" ${CMAKE_PREFIX_PATH})

project(charles_mesh)

add_executable(
    ${PROJECT_NAME}_test
    mesh_factory.cpp
    charles_mesh.cpp
    charles_bvh.cpp
    simple_math.cpp
    mesh_type_test.cpp
    unit_test.cpp
    charles_mesh_test.cpp)
find_package(GTest CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME}_test GTest::gtest GTest::gmock)
