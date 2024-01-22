add_executable(test_gtw_mqtt ./test_gtw_mqtt.c)
target_link_libraries(test_gtw_mqtt ufr_gtw_mqtt)
add_test(NAME test_gtw_mqtt COMMAND test_gw_mqtt)
