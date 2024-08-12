add_executable(publisher 0-basic/publisher.c)
target_link_libraries(publisher ufr)
install(TARGETS publisher DESTINATION examples)

add_executable(subscriber 0-basic/subscriber.c)
target_link_libraries(subscriber ufr)
install(TARGETS subscriber DESTINATION examples)

add_executable(server 0-basic/server.c)
target_link_libraries(server ufr cwalk)
install(TARGETS server DESTINATION examples)

add_executable(client 0-basic/client.c)
target_link_libraries(client ufr cwalk)
install(TARGETS client DESTINATION examples)

add_executable(publisher_2topic 0-basic/publisher_2topic.c)
target_link_libraries(publisher_2topic ufr)
install(TARGETS publisher_2topic DESTINATION examples)

add_executable(subscriber_2topic 0-basic/subscriber_2topic.c)
target_link_libraries(subscriber_2topic ufr)
install(TARGETS subscriber_2topic DESTINATION examples)


find_package(OpenCV)
if(OpenCV_DIR)
    add_executable(publisher_image 0-basic/publisher_image.cpp)
    target_link_libraries(publisher_image ufr ${OpenCV_LIBS})
    target_include_directories(publisher_image PUBLIC "${OPENCV_INCLUDE_DIR}")
    install(TARGETS publisher_image DESTINATION examples)

    add_executable(subscriber_image 0-basic/subscriber_image.cpp)
    target_link_libraries(subscriber_image ufr ${OpenCV_LIBS})
    target_include_directories(subscriber_image PUBLIC "${OPENCV_INCLUDE_DIR}")
    install(TARGETS subscriber_image DESTINATION examples)
endif()