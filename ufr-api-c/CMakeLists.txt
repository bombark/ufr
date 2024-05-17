cmake_minimum_required(VERSION 3.5)
project(ufr)

include_directories(./include)

add_library(ufr SHARED
    # core
    ./src/ufr_api.c
    ./src/ufr_sys.c
    ./src/ufr_sys_std_enc.c
    ./src/ufr_sys_std_dcr.c
    )

# target_compile_options(lt_api PUBLIC -O1 -Wall -Wfloat-conversion)
target_compile_options(ufr PUBLIC -O1)
target_link_libraries(ufr dl)

install(TARGETS ufr DESTINATION lib)
install(FILES ./include/ufr.h DESTINATION include)

# test
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    enable_testing()
    add_subdirectory(./tests)
endif()
