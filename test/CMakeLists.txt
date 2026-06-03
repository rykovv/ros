include(${CMAKE_SOURCE_DIR}/cmake/get_cpm.cmake)

CPMAddPackage(
    NAME googletest
    GITHUB_REPOSITORY google/googletest
    VERSION 1.14.0
    OPTIONS "INSTALL_GTEST OFF"
)

enable_testing()

add_subdirectory(unit)
add_subdirectory(compile_fail)
