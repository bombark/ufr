cmake_minimum_required(VERSION 3.0)
project(vri-nav-simple)

add_executable(vri-nav
    ./src/main.c 
    )

target_link_libraries(vri-nav ufr)

install(TARGETS vri-nav DESTINATION bin)