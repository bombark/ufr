cmake_minimum_required(VERSION 3.0)
project(ufr)

include_directories(./include)

add_library(ufr SHARED
    # core
    ./src/ufr_api.c
    ./src/ufr_sys.c
    )

# target_compile_options(lt_api PUBLIC -O1 -Wall -Wfloat-conversion)
target_compile_options(ufr PUBLIC -O1)

install(TARGETS ufr DESTINATION lib)
install(FILES ./include/ufr.h DESTINATION include)

# test
enable_testing()
add_subdirectory(./tests)
