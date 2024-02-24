add_executable(test_enc_msgpack ./test_enc_msgpack.c)
target_link_libraries(test_enc_msgpack ufr_enc_msgpack)
add_test(NAME test_enc_msgpack COMMAND test_enc_msgpack)

add_executable(test_dcr_msgpack ./test_dcr_msgpack.c)
target_link_libraries(test_dcr_msgpack ufr_dcr_msgpack)
add_test(NAME test_dcr_msgpack COMMAND test_dcr_msgpack)