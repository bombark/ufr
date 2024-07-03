add_executable(publisher 0-basic/publisher.c)
target_link_libraries(publisher ufr)
install(TARGETS publisher DESTINATION examples)

add_executable(subscriber 0-basic/subscriber.c)
target_link_libraries(subscriber ufr)
install(TARGETS subscriber DESTINATION examples)

add_executable(server 0-basic/server.c)
target_link_libraries(server ufr)
install(TARGETS server DESTINATION examples)

add_executable(client 0-basic/client.c)
target_link_libraries(client ufr)
install(TARGETS client DESTINATION examples)


# find_package(OpenCV REQUIRED )
# add_executable(publisher_image 0-basic/publisher_image.cpp)
# target_link_libraries(publisher_image ufr ${OpenCV_LIBS})
# target_include_directories(publisher_image PUBLIC "${OPENCV_INCLUDE_DIR}")