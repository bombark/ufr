cmake_minimum_required(VERSION 3.5)
project(ufr-api-c)

# Coverage test
# SET(CMAKE_C_FLAGS "-fprofile-arcs -ftest-coverage --coverage -g -O0")
# SET(GCC_COVERAGE_LINK_FLAGS "-lgcov")

include_directories(./include)

add_library(ufr SHARED
    # core
    ./src/ufr_api_get.c
    ./src/ufr_api_put.c
    ./src/ufr_api.c
    ./src/ufr_sys.c
    ./src/ufr_linux.c
    ./src/ufr_sys_std_enc.c
    ./src/ufr_sys_std_dcr.c
    ./src/ufr_posix_pipe.c
    ./src/ufr_buffer.c
    )

target_compile_options(ufr PUBLIC -O3 -Wall -Wfloat-conversion)
target_link_libraries(ufr dl)

install(TARGETS ufr DESTINATION lib)
install(FILES ./include/ufr.h DESTINATION include)

# test
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    enable_testing()
    add_subdirectory(./tests)
# endif()

add_custom_target(coverage
    COMMAND gcovr ${PROJECT_BINARY_DIR}
    )

add_subdirectory(./examples)
