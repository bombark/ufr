cmake_minimum_required(VERSION 3.0)
project(ufr)

# include UFR API
include_directories(./ufr-api/include)

# Build libraries with no dependencies
add_subdirectory(./ufr-api)
add_subdirectory(./ufr-gtw-posix)
add_subdirectory(./ufr-cc-std)

# Configure compilation
option(BUILD_UFR_GTW_ZMQ     "build the gateway for ZMQ"    OFF)
option(BUILD_UFR_GTW_MQTT    "build the gateway for MQTT"   OFF)
option(BUILD_UFR_CC_MSGPACK  "build the coder for Msgpack"  OFF)
option(BUILD_UFR_STK_SQLITE  "build the stack for Sqlite3"  OFF)
option(BUILD_EXAMPLES        "build the examples"           ON )
option(BUILD_TESTS           "build the tests"              ON )
# option(BUILD_APPLICATION        "build the applications"       ON )

# Build libraries with dependencies
if(BUILD_UFR_GTW_ZMQ)
    # debian/ubuntu: sudo apt install libzmq3-dev
    add_subdirectory(./ufr-gtw-zmq)
endif(BUILD_UFR_GTW_ZMQ)

if(BUILD_UFR_GTW_MQTT)
    # debian/ubuntu: sudo apt install libmosquitto-dev
    add_subdirectory(./ufr-gtw-mqtt)
endif(BUILD_UFR_GTW_MQTT)

if(BUILD_UFR_CC_MSGPACK)
    # debian/ubuntu: sudo apt install libmsgpack-dev
    add_subdirectory(./ufr-cc-msgpack)
endif(BUILD_UFR_CC_MSGPACK)

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
endif(BUILD_TESTS)