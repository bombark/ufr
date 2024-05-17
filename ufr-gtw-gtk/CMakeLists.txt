cmake_minimum_required(VERSION 3.5)
project(ufr-gtw-gtk)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK4 REQUIRED IMPORTED_TARGET gtk4)

add_library(gtw_gtk SHARED
    ./src/ufr_gtw_gtk.c
    )
target_compile_options(gtw_gtk PUBLIC -O1)
target_link_libraries(gtw_gtk ufr PkgConfig::GTK4)
install(TARGETS gtw_gtk DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)