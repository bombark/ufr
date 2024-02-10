cmake_minimum_required(VERSION 3.0)
project(vri-base-pioneer-webots)

link_directories(/usr/local/webots/lib/controller/)
include_directories(/usr/local/webots/include/controller/c/)
add_executable(vri-base-webots src/main.c)
target_link_libraries(vri-base-webots m Controller ufr)
install(TARGETS vri-base-webots DESTINATION bin)
