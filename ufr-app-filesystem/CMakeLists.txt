cmake_minimum_required(VERSION 3.0)
project(ufr-app-filesystem)

# Server
add_executable(ufr_data ./src/main.c)
target_link_libraries(ufr_data ufr)
install(TARGETS ufr_data DESTINATION bin)

# Shell Client
# add_executable(ufr_shell ./src/shell.c)
# target_link_libraries(ufr_shell ufr cwalk)
# install(TARGETS ufr_shell DESTINATION bin)
