cmake_minimum_required(VERSION 3.0)
project(cc_msgpack)

# find_package(msgpackc REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(msgpackc IMPORTED_TARGET libmsgpackc REQUIRED)

add_library(ufr_enc_msgpack SHARED ./src/ufr_enc_msgpack.c)
target_link_libraries(ufr_enc_msgpack msgpackc ufr)
target_compile_options(ufr_enc_msgpack PUBLIC -O1)
install(TARGETS ufr_enc_msgpack DESTINATION lib)

add_library(ufr_dcr_msgpack SHARED ./src/ufr_dcr_msgpack.c)
target_link_libraries(ufr_dcr_msgpack msgpackc ufr)
target_compile_options(ufr_dcr_msgpack PUBLIC -O1)
install(TARGETS ufr_dcr_msgpack DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)