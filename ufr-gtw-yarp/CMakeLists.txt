cmake_minimum_required(VERSION 3.5)
project(gw_zmq)


add_library(gw_zmq SHARED
    ./src/lt_zmq_common.c 
    ./src/lt_zmq_topic.c 
    ./src/lt_zmq_socket.c 
    )

target_compile_options(gw_zmq PUBLIC -O1)
target_link_libraries(gw_zmq zmq lt_api)

install(TARGETS gw_zmq DESTINATION lib)

# test
# enable_testing()
# add_subdirectory(./tests)