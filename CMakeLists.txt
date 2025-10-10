cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 20)
set(LIBIGL_WITH_CGAL ON)
set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
# set(CMAKE_PREFIX_PATH "D:\\OwnLibrary" ${CMAKE_PREFIX_PATH})
list(APPEND CMAKE_PREFIX_PATH "D:/OwnLibrary")

project(charles_mesh)

find_package(PkgConfig REQUIRED)
pkg_check_modules(gmp REQUIRED IMPORTED_TARGET gmp)
# target_link_libraries(hello_world PkgConfig::gmp)

find_package(SymEngine CONFIG REQUIRED)
find_package(hello_symengine)
find_package(Boost COMPONENTS functional REQUIRED)
find_package(triangles_shower)

# target_link_libraries(hello_world symengine)

add_executable(
    ${PROJECT_NAME}_test
    mesh_factory.cpp
    charles_mesh.cpp
    charles_bvh.cpp
    simple_math.cpp
    quadric_error_metrics.cpp
    basic_type.cpp
    mesh_type_test.cpp
    unit_test.cpp
    charles_mesh_test.cpp
    charles_cgal_test.cpp
    draw_mesh_test.cpp
)
find_package(GTest CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME}_test GTest::gtest GTest::gmock PkgConfig::gmp symengine hello_symengine::hello_symengine Boost::functional triangles_shower::triangles_shower)

add_executable(
    mesh_viewer
    mesh_factory.cpp
    charles_mesh.cpp
    charles_bvh.cpp
    simple_math.cpp
    quadric_error_metrics.cpp
    basic_type.cpp
    viewer_middle_layer.cpp
)

find_package(hello_qt)

target_link_libraries(mesh_viewer triangles_shower::triangles_shower hello_qt::hello_qt GTest::gtest GTest::gmock PkgConfig::gmp symengine hello_symengine::hello_symengine Boost::functional)
# Deploy LibraryA's config files to executable directory
triangles_shower_deploy_config_files(mesh_viewer)

# this is for export libary for external use
set(CMAKE_INSTALL_PREFIX "D:/OwnLibrary")

add_library(
    ${PROJECT_NAME}
    mesh_factory.cpp
    charles_mesh.cpp
    charles_bvh.cpp
    simple_math.cpp
    quadric_error_metrics.cpp
    basic_type.cpp
)

target_link_libraries(${PROJECT_NAME} GTest::gtest GTest::gmock PkgConfig::gmp symengine hello_symengine::hello_symengine Boost::functional)

# 引入要用到的 CMake 模块
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# 基本安装及 Targets 文件的生成
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Targets 文件的安装
install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Config 文件的导出（与安装？）
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# ConfigVersion 文件的导出
write_basic_package_version_file(
    ${PROJECT_NAME}-config-version.cmake
    VERSION 0.1
    COMPATIBILITY AnyNewerVersion
)

# Config 和 ConfigVersion 文件的安装
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${PROJECT_SOURCE_DIR}/charles_bvh.h" DESTINATION include)
install(FILES "${PROJECT_SOURCE_DIR}/charles_mesh.h" DESTINATION include)
install(FILES "${PROJECT_SOURCE_DIR}/mesh_factory.h" DESTINATION include)
install(FILES "${PROJECT_SOURCE_DIR}/basic_type.h" DESTINATION include)
install(FILES "${PROJECT_SOURCE_DIR}/simple_math.h" DESTINATION include)
install(FILES "${PROJECT_SOURCE_DIR}/charles_cgal.h" DESTINATION include)
install(FILES "${PROJECT_SOURCE_DIR}/quadric_error_metrics.h" DESTINATION include)
