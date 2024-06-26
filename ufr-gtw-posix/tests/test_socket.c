#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <ufr.h>

int ufr_gtw_posix_new_socket(link_t* link, int type);

void test() {
    link_t link;
    assert( ufr_gtw_posix_new_socket(&link, UFR_START_SERVER) == UFR_OK );
    assert( ufr_boot_gtw(&link, NULL) == UFR_OK );
    assert( ufr_start_server(&link, NULL) == UFR_OK );
    assert( ufr_link_is_server(&link) == true );

    while(1) {
        assert( ufr_recv(&link) == true );

        char* text = "HTTP/1.1 200\r\n"
            "Content-Type: text/plain\r\n"
            "\r\n"
            "OK e dai ?\n";

        ufr_write(&link, text, strlen(text));
        ufr_send(&link);
    }

    ufr_close(&link);
}

void test2() {
    link_t link;
    assert( ufr_gtw_posix_new_socket(&link, UFR_START_SERVER) == UFR_OK );
    assert( ufr_boot_gtw(&link, NULL) == UFR_OK );
    assert( ufr_start_server(&link, NULL) == UFR_OK );
    assert( ufr_link_is_server(&link) == true );

    while(1) {
        link_t client;
        // client = ufr_accept(&link);

        char* text = "HTTP/1.1 200\r\n"
            "Content-Type: text/plain\r\n"
            "\r\n"
            "OK e dai ?\n";

        ufr_write(&client, text, strlen(text));
        ufr_close(&client);
    }

    ufr_close(&link);
}


int main() {
    // test();
    return 0;
}