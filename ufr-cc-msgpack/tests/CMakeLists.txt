add_executable(test_ecr_msgpack ./test_ecr_msgpack.c)
target_link_libraries(test_ecr_msgpack ufr_ecr_msgpack)
add_test(NAME test_ecr_msgpack COMMAND test_ecr_msgpack)

add_executable(test_dcr_msgpack ./test_dcr_msgpack.c)
target_link_libraries(test_dcr_msgpack ufr_dcr_msgpack)
add_test(NAME test_dcr_msgpack COMMAND test_dcr_msgpack)