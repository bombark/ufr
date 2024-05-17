cmake_minimum_required(VERSION 3.5)
project(ufr-gtw-posix)


add_library(ufr_gtw_posix SHARED
    ./src/ufr_gtw_posix.c
    ./src/ufr_posix_dir.c 
    ./src/ufr_posix_file.c 
    ./src/ufr_posix_pipe.c 
    ./src/ufr_posix_socket.c 
    ./src/ufr_posix_socket_srv.c
    ./src/ufr_posix_socket_cli.c
    ./src/ufr_message.c
    )

target_compile_options(ufr_gtw_posix PUBLIC -O1)
target_link_libraries(ufr_gtw_posix ufr)
install(TARGETS ufr_gtw_posix DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)