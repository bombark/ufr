cmake_minimum_required(VERSION 3.0)
project(ufr-dae-std)

# libenc_csv.so
add_library(ufr_enc_csv SHARED ./src/ufr_enc_csv.cpp)
target_compile_options(ufr_enc_csv PUBLIC -O1)
target_link_libraries(ufr_enc_csv ufr)
install(TARGETS ufr_enc_csv DESTINATION lib)

# libdcr_csv.so
add_library(ufr_dcr_csv SHARED ./src/ufr_dcr_csv.c)
target_compile_options(ufr_dcr_csv PUBLIC -O1)
target_link_libraries(ufr_dcr_csv ufr)
install(TARGETS ufr_dcr_csv DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)