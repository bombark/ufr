cmake_minimum_required(VERSION 3.0)
project(ufr-app-filesystem)

add_executable(ufr-app-filesystem
    ./src/main.c 
    )

target_link_libraries(ufr-app-filesystem ufr)

install(TARGETS ufr-app-filesystem DESTINATION bin)