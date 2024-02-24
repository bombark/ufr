add_executable(test_enc_std ./test_enc_std.c)
target_link_libraries(test_enc_std ufr_enc_std)
add_test(NAME test_enc_std COMMAND test_enc_std)

# add_test(NAME test_valgrind_ecr_std COMMAND valgrind test_ecr_std)

add_executable(test_dcr_std ./test_dcr_std.c)
target_link_libraries(test_dcr_std ufr_dcr_std)
add_test(NAME test_dcr_std COMMAND test_dcr_std)

# add_test(NAME test_valgrind_dcr_std COMMAND valgrind test_dcr_std)