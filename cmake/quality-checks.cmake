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

find_program(CLANG_FORMAT NAMES clang-format clang-format-21 clang-format-20
                                clang-format-19 clang-format-18)
if(CLANG_FORMAT)
    file(GLOB_RECURSE ROS_FORMAT_FILES
        ${CMAKE_SOURCE_DIR}/include/*.hpp
        ${CMAKE_SOURCE_DIR}/test/*.hpp
        ${CMAKE_SOURCE_DIR}/test/*.cpp)

    # Rewrite sources in place to match .clang-format
    add_custom_target(clang-format
        COMMAND ${CLANG_FORMAT} -i --style=file ${ROS_FORMAT_FILES}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Formatting sources with clang-format"
    )

    # Fail (non-zero exit) if any source is not formatted; changes nothing
    add_custom_target(clang-format-check
        COMMAND ${CLANG_FORMAT} --dry-run --Werror --style=file ${ROS_FORMAT_FILES}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Checking formatting with clang-format"
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
