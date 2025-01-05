cmake_minimum_required(VERSION 3.5)
project(ufr-stk-ssh)

include_directories(./src)

add_library(ufr_gtw_ssh SHARED
    ./src/ufr_gtw_ssh.c
    )

target_compile_options(ufr_gtw_ssh PUBLIC -O1)
target_link_libraries(ufr_gtw_ssh ufr ssh)

install(TARGETS ufr_gtw_ssh DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)