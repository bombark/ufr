cmake_minimum_required(VERSION 3.5)
project(ufr-gtw-qt)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(ufr_gtw_qt SHARED
    ./src/ufr_gtw_qt.cpp
    )
target_compile_options(ufr_gtw_qt PUBLIC -O1)
target_link_libraries(ufr_gtw_qt ufr  Qt5::Widgets)
install(TARGETS ufr_gtw_qt DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)