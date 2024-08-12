cmake_minimum_required(VERSION 3.0)
project(ufr-app-filesystem)

# Server
add_executable(ufr-sys-data ./src/main.c)
target_link_libraries(ufr-sys-data ufr)
install(TARGETS ufr-sys-data DESTINATION bin)

# Shell Client
# add_executable(ufr-shell ./src/shell.c)
# target_link_libraries(ufr-shell ufr cwalk)
# install(TARGETS ufr-shell DESTINATION bin)