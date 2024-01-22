cmake_minimum_required(VERSION 3.0)
project(ufr-app-proxy)

add_executable(ufr_proxy ./src/ufr_proxy.c)
target_link_libraries(ufr_proxy ufr)
target_compile_options(ufr_proxy PUBLIC -O1)
install(TARGETS ufr_proxy DESTINATION bin)

# test
# enable_testing()
# add_subdirectory(./tests)