cmake_minimum_required(VERSION 3.5)
project(urf-gtw-mqtt)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)

add_library(ufr_gtw_mqtt SHARED
    ./src/urf_gtw_mqtt.c
    )

target_compile_options(ufr_gtw_mqtt PUBLIC -O1)
target_link_libraries(ufr_gtw_mqtt PkgConfig::Mosquitto ufr)

install(TARGETS ufr_gtw_mqtt DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)