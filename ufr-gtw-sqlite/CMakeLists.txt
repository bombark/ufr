cmake_minimum_required(VERSION 3.0)
project(ufr-gtw-sqlite)


add_library(ufr_gtw_sqlite SHARED
    ./src/ufr_gtw_sqlite.c
    ./src/ufr_ecr_sqlite.c
    ./src/ufr_dcr_sqlite.c
    )

target_compile_options(ufr_gtw_sqlite PUBLIC -O1)
target_link_libraries(ufr_gtw_sqlite sqlite3 ufr)

install(TARGETS ufr_gtw_sqlite DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)