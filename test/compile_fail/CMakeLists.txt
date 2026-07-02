set(COMPILE_FAIL_TESTS
    write_ro_field
    read_wo_register
    field_value_exceeds_length
    assign_ro_register_bits
    write_non_writable_field
    read_non_readable_field
    duplicate_field_in_apply
    duplicate_register_in_apply
    register_value_too_wide
)

foreach(test_name ${COMPILE_FAIL_TESTS})
    add_library(compile_fail_${test_name} OBJECT EXCLUDE_FROM_ALL ${test_name}.cpp)
    target_link_libraries(compile_fail_${test_name} PRIVATE ros)

    add_test(
        NAME compile_fail/${test_name}
        COMMAND ${CMAKE_COMMAND}
            --build ${CMAKE_BINARY_DIR}
            --target compile_fail_${test_name}
            --config $<CONFIG>
    )
    set_tests_properties(compile_fail/${test_name} PROPERTIES WILL_FAIL TRUE)
endforeach()
