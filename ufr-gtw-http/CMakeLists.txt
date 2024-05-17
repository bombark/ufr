cmake_minimum_required(VERSION 3.5)
project(gtw_http)


add_library(gtw_http SHARED
    ./src/gtw_httpd.c    
    )
target_compile_options(gtw_http PUBLIC -O1)
target_link_libraries(gtw_http ufr_api microhttpd)
install(TARGETS gtw_http DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)