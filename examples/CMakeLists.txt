add_executable(publisher 0-basic/publisher.c)
target_link_libraries(publisher ufr)

add_executable(subscriber 0-basic/subscriber.c)
target_link_libraries(subscriber ufr)

add_executable(server 0-basic/server.c)
target_link_libraries(server ufr)

add_executable(client 0-basic/client.c)
target_link_libraries(client ufr)