cmake_minimum_required(VERSION 3.5)
project(ufr)

# Check msgpack library
set(HAVE_MSGPACK OFF)
find_package(msgpack)
if (msgpack_DIR)
    set(HAVE_MSGPACK ON)
endif()

# Check mosquitto library
find_package(PkgConfig)
set(HAVE_MOSQUITTO OFF)
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto)
if (Mosquitto_FOUND)
    set(HAVE_MOSQUITTO ON)
endif()

# Check zmq library
set(HAVE_ZMQ OFF)
pkg_check_modules(ZMQ IMPORTED_TARGET libzmq)
if (ZMQ_FOUND)
    set(HAVE_ZMQ ON)
endif()


# Configure compilation
option(BUILD_UFR_GTW_ZMQ     "build the gateway for ZMQ"    ${HAVE_ZMQ})
option(BUILD_UFR_GTW_MQTT    "build the gateway for MQTT"   ${HAVE_MOSQUITTO})
option(BUILD_UFR_CDR_MSGPACK "build the coder for MsgPack"  ${HAVE_MSGPACK})
option(BUILD_UFR_STK_SQLITE  "build the stack for Sqlite3"  ON )
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
# add_subdirectory(./ufr-cdr-std)

# Build libraries with dependencies
if(BUILD_UFR_GTW_ZMQ)
    # debian/ubuntu: sudo apt install libzmq3-dev
    # add_subdirectory(./ufr-gtw-zmq)
endif(BUILD_UFR_GTW_ZMQ)

if(BUILD_UFR_GTW_MQTT)
    # debian/ubuntu: sudo apt install libmosquitto-dev
    add_subdirectory(./ufr-gtw-mqtt)
endif(BUILD_UFR_GTW_MQTT)

if(BUILD_UFR_CDR_MSGPACK)
    # debian/ubuntu: sudo apt install libmsgpack-dev
    # add_subdirectory(./ufr-cdr-msgpack)
endif(BUILD_UFR_CDR_MSGPACK)

if(BUILD_UFR_STK_SQLITE)
    # debian/ubuntu: sudo apt install libsqlite3-dev
    # add_subdirectory(./ufr-stk-sqlite)
endif(BUILD_UFR_STK_SQLITE)

# Build examples
if(BUILD_EXAMPLES)
    add_subdirectory(./examples/)
endif(BUILD_EXAMPLES)

# Build tests
if(BUILD_TESTS)
    # add_subdirectory(./tests/)
    # enable_testing()
    # add_test(NAME test_gtw_zmq COMMAND ./tests/test_topic "@new zmq:topic @port 2000")
endif(BUILD_TESTS)


# add_subdirectory(./rplidar_ros)

# add_subdirectory(./ufr-app-filesystem)
# add_subdirectory(./ufr-app-proxy)

# add_subdirectory(./ufr-gtw-qt)

# add_subdirectory(./ufr-cdr-http)
# add_subdirectory(./ufr-gtw-http)

# add_subdirectory(./ufr-stk-ssh)

# add_subdirectory(ufr-stk-irrlicht)

# add_subdirectory(./ufr-stk-ros_humble)
# add_subdirectory(./ufr-stk-ros_melodic)
# add_subdirectory(./ufr-stk-ros_noetic)
# add_subdirectory(./ufr-stk-webots)

# add_subdirectory(./ufr-dev-camera)


# install(FILES ./etc/init.bash DESTINATION .)
# install(FILES ./etc/Makefile DESTINATION .)

