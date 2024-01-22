add_executable(test_ecr_std ./test_ecr_std.c)
target_link_libraries(test_ecr_std ufr_ecr_std)
add_test(NAME test_ecr_std COMMAND test_ecr_std)

# add_test(NAME test_valgrind_ecr_std COMMAND valgrind test_ecr_std)

add_executable(test_dcr_std ./test_dcr_std.c)
target_link_libraries(test_dcr_std ufr_dcr_std)
add_test(NAME test_dcr_std COMMAND test_dcr_std)

# add_test(NAME test_valgrind_dcr_std COMMAND valgrind test_dcr_std)