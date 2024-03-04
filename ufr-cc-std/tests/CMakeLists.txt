add_executable(test_enc_csv ./test_enc_std.c)
target_link_libraries(test_enc_csv ufr_enc_csv)
add_test(NAME test_enc_csv COMMAND test_enc_csv)

# add_test(NAME test_valgrind_ecr_std COMMAND valgrind test_ecr_std)

add_executable(test_dcr_csv ./test_dcr_std.c)
target_link_libraries(test_dcr_csv ufr_dcr_csv)
add_test(NAME test_dcr_csv COMMAND test_dcr_csv)

# add_test(NAME test_valgrind_dcr_std COMMAND valgrind test_dcr_std)