cmake_minimum_required(VERSION 3.0)
project(gw_http)


add_library(gw_http SHARED
    ./src/gw_httpd.c    
    )
target_compile_options(gw_http PUBLIC -O1)
target_link_libraries(gw_http lt_api microhttpd)
install(TARGETS gw_http DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)