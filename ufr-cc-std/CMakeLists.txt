cmake_minimum_required(VERSION 3.0)
project(ufr-dae-std)

# libecr_std.so
add_library(ufr_ecr_std SHARED ./src/ufr_ecr_std.cpp)
target_compile_options(ufr_ecr_std PUBLIC -O1)
target_link_libraries(ufr_ecr_std ufr)
install(TARGETS ufr_ecr_std DESTINATION lib)

# libdcr_std.so
add_library(ufr_dcr_std SHARED ./src/ufr_dcr_std.c)
target_compile_options(ufr_dcr_std PUBLIC -O1)
target_link_libraries(ufr_dcr_std ufr)
install(TARGETS ufr_dcr_std DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)