cmake_minimum_required(VERSION 3.5)
project(cdr_http)

add_library(ufr_enc_http SHARED ./src/ufr_enc_http_req.c ./src/ufr_enc_http_ans.c ./src/picohttpparser.c)
target_link_libraries(ufr_enc_http ufr)
target_compile_options(ufr_enc_http PUBLIC -O1)
install(TARGETS ufr_enc_http DESTINATION lib)

add_library(ufr_dcr_http SHARED ./src/ufr_dcr_http_req.c ./src/ufr_dcr_http_ans.c ./src/picohttpparser.c)
target_link_libraries(ufr_dcr_http ufr)
target_compile_options(ufr_dcr_http PUBLIC -O1)
install(TARGETS ufr_dcr_http DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)