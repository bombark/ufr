cmake_minimum_required(VERSION 3.0)
project(ufr-gtw-zmq)


add_library(ufr_gtw_zmq SHARED
    ./src/ufr_zmq_common.c 
    ./src/ufr_zmq_topic.c 
    ./src/ufr_zmq_socket.c 
    )

target_compile_options(ufr_gtw_zmq PUBLIC -O1)
target_link_libraries(ufr_gtw_zmq zmq ufr)

install(TARGETS ufr_gtw_zmq DESTINATION lib)

# test
# enable_testing()
# add_subdirectory(./tests)