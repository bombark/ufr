cmake_minimum_required(VERSION 3.0)
project(vri-base-pioneer)

include_directories(src/rplidar)
include_directories(src/rplidar/arch/linux/)
include_directories(src/rplidar/dataunpacker/)


add_executable( vri-base
	src/rplidar/arch/linux/net_socket.cpp
	src/rplidar/arch/linux/net_serial.cpp
	src/rplidar/arch/linux/timer.cpp

	src/rplidar/dataunpacker/dataunpacker.cpp
	src/rplidar/dataunpacker/unpacker/handler_normalnode.cpp
	src/rplidar/dataunpacker/unpacker/handler_hqnode.cpp
	src/rplidar/dataunpacker/unpacker/handler_capsules.cpp

	src/rplidar/hal/thread.cpp

	src/rplidar/rplidar_driver.cpp
	src/rplidar/sl_crc.cpp
	src/rplidar/sl_lidar_driver.cpp
	src/rplidar/sl_serial_channel.cpp
	src/rplidar/sl_async_transceiver.cpp
	src/rplidar/sl_lidarprotocol_codec.cpp
	src/rplidar/sl_serial_channel.cpp
	src/rplidar/sl_tcp_channel.cpp
	src/rplidar/sl_udp_channel.cpp

	src/main.cpp
	)

# add_executable( vri-base-lidar
#	src/main_lidar.cpp
#	)

target_link_libraries(vri-base ufr Aria)
install(TARGETS vri-base DESTINATION bin)

# target_link_libraries(vri-base-lidar ufr)
# install(TARGETS vri-base-lidar DESTINATION bin)
