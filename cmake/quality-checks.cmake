if(DEFINED ROS_SANITIZER)
    add_compile_options(-fsanitize=${ROS_SANITIZER} -fno-omit-frame-pointer)
    add_link_options(-fsanitize=${ROS_SANITIZER})
endif()

include(ProcessorCount)
ProcessorCount(NPROC)
if(NPROC EQUAL 0)
    set(NPROC 1)
endif()

find_program(RUN_CLANG_TIDY NAMES run-clang-tidy run-clang-tidy-18 run-clang-tidy-19 run-clang-tidy-20 run-clang-tidy-21)
if(RUN_CLANG_TIDY)
    add_custom_target(clang-tidy
        COMMAND ${RUN_CLANG_TIDY} -p ${CMAKE_BINARY_DIR} "test/unit/.*\\.cpp"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ros_tests
        COMMENT "Running clang-tidy"
    )
endif()

find_program(VALGRIND_EXECUTABLE NAMES valgrind)
if(VALGRIND_EXECUTABLE)
    add_custom_target(valgrind
        COMMAND ${CMAKE_CTEST_COMMAND}
            --output-on-failure -j ${NPROC}
            --overwrite MemoryCheckCommand=${VALGRIND_EXECUTABLE}
            --overwrite "MemoryCheckCommandOptions=--leak-check=full --error-exitcode=1"
            -T memcheck
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
        DEPENDS ros_tests
        COMMENT "Running valgrind memcheck"
    )
endif()
