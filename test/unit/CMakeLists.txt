add_executable(ros_tests
    test_field.cpp
    test_reg.cpp
    test_apply_field.cpp
    test_apply_reg.cpp
    test_literals.cpp
    test_constexpr.cpp
)

target_link_libraries(ros_tests PRIVATE ros GTest::gtest_main)
target_include_directories(ros_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

include(GoogleTest)
gtest_discover_tests(ros_tests)
