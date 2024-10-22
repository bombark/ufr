cmake_minimum_required(VERSION 3.5)
project(ufr-dev-lidar)

include_directories(
    ./sdk/include
    ./sdk/arch/linux/
    ./sdk/src/)

add_library(lidar_sdk STATIC
    ./sdk/src/sl_lidar_driver.cpp
    ./sdk/src/hal/thread.cpp
    ./sdk/src/sl_crc.cpp
    ./sdk/src/sl_serial_channel.cpp
    ./sdk/src/sl_lidarprotocol_codec.cpp
    ./sdk/src/sl_async_transceiver.cpp
    ./sdk/src/sl_tcp_channel.cpp
    ./sdk/src/sl_udp_channel.cpp
    ./sdk/src/dataunpacker/dataunpacker.cpp
    ./sdk/src/dataunpacker/unpacker/handler_hqnode.cpp
    ./sdk/src/arch/linux/net_socket.cpp
    ./sdk/src/dataunpacker/unpacker/handler_capsules.cpp
    ./sdk/src/arch/linux/timer.cpp
    ./sdk/src/dataunpacker/unpacker/handler_normalnode.cpp
    ./sdk/src/arch/linux/net_serial.cpp
)

add_executable(ufr-dev-lidar ./src/main.cpp)

target_link_libraries(ufr-dev-lidar ufr lidar_sdk)