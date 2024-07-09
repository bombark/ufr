cmake_minimum_required(VERSION 3.0)
project(ufr-app-filesystem)

# Server
add_executable(ufr-app-filesystem ./src/main.c)
target_link_libraries(ufr-app-filesystem ufr cwalk)
install(TARGETS ufr-app-filesystem DESTINATION bin)

# Shell Client
add_executable(ufr-shell ./src/shell.c)
target_link_libraries(ufr-shell ufr cwalk)
install(TARGETS ufr-shell DESTINATION bin)