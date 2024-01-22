cmake_minimum_required(VERSION 3.0)
project(urf-gtw-mqtt)


add_library(ufr_gtw_mqtt SHARED
    ./src/urf_gtw_mqtt.c
    )

target_compile_options(ufr_gtw_mqtt PUBLIC -O1)
target_link_libraries(ufr_gtw_mqtt mosquitto ufr)

install(TARGETS ufr_gtw_mqtt DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)