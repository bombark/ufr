cmake_minimum_required(VERSION 3.5)
project(ufr-gtw-gtk)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK4 REQUIRED IMPORTED_TARGET gtk4)

add_library(ufr_gtw_gtk SHARED
    ./src/ufr_gtw_gtk.c
    )
target_compile_options(ufr_gtw_gtk PUBLIC -O1)
target_link_libraries(ufr_gtw_gtk ufr PkgConfig::GTK4)
install(TARGETS ufr_gtw_gtk DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)