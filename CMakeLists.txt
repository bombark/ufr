cmake_minimum_required(VERSION 3.5)
project(ufr)


# Configure compilation
option(BUILD_UFR_GTW_ZMQ     "build the gateway for ZMQ"    OFF)
option(BUILD_UFR_GTW_MQTT    "build the gateway for MQTT"   OFF)
option(BUILD_UFR_CDR_MSGPACK "build the coder for MsgPack"  OFF)
option(BUILD_UFR_STK_SQLITE  "build the stack for Sqlite3"  OFF)
option(BUILD_EXAMPLES        "build the examples"           ON )
option(BUILD_TESTS           "build the tests"              ON )
option(COVERAGE_TEST         "coverage test"                OFF)
# option(BUILD_APPLICATION        "build the applications"       ON )


if(COVERAGE_TEST)
    SET(CMAKE_C_FLAGS "-fprofile-arcs -ftest-coverage --coverage -g -O0")
    SET(GCC_COVERAGE_LINK_FLAGS "-lgcov")
    # gcovr -r ..
endif(COVERAGE_TEST)

# include UFR API
include_directories(./ufr-api-c/include)

# Build libraries with no dependencies
add_subdirectory(./ufr-api-c)
add_subdirectory(./ufr-gtw-posix)
add_subdirectory(./ufr-cdr-std)

# Build libraries with dependencies
if(BUILD_UFR_GTW_ZMQ)
    # debian/ubuntu: sudo apt install libzmq3-dev
    add_subdirectory(./ufr-gtw-zmq)
endif(BUILD_UFR_GTW_ZMQ)

if(BUILD_UFR_GTW_MQTT)
    # debian/ubuntu: sudo apt install libmosquitto-dev
    add_subdirectory(./ufr-gtw-mqtt)
endif(BUILD_UFR_GTW_MQTT)

if(BUILD_UFR_CDR_MSGPACK)
    # debian/ubuntu: sudo apt install libmsgpack-dev
    add_subdirectory(./ufr-cdr-msgpack)
endif(BUILD_UFR_CDR_MSGPACK)

if(BUILD_UFR_STK_SQLITE)
    # debian/ubuntu: sudo apt install libsqlite3-dev
    add_subdirectory(./ufr-stk-sqlite)
endif(BUILD_UFR_STK_SQLITE)

# Build examples
if(BUILD_EXAMPLES)
    add_subdirectory(./examples/)
endif(BUILD_EXAMPLES)

# Build tests
if(BUILD_TESTS)
    add_subdirectory(./tests/)
    enable_testing()
    # add_test(NAME test_gtw_zmq COMMAND ./tests/test_topic "@new zmq:topic @port 2000")
endif(BUILD_TESTS)


add_subdirectory(./ufr-app-filesystem)
add_subdirectory(./ufr-app-proxy)

install(FILES ./etc/init.bash DESTINATION .)
install(FILES ./etc/Makefile DESTINATION .)
