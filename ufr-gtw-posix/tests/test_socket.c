#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <ufr.h>

int ufr_new_gtw_posix_socket(link_t* link, const lt_args_t* args);

void test() {
    link_t link;
    assert( ufr_new_gtw_posix_socket(&link, NULL) == LT_OK );
    assert( ufr_start_bind(&link, NULL) == LT_OK );
    assert( ufr_link_is_server(&link) == true );

    while(1) {
        assert( lt_recv(&link) == true );

        char* text = "HTTP/1.1 200\r\n"
            "Content-Type: text/plain\r\n"
            "\r\n"
            "OK e dai ?\n";

        lt_write(&link, text, strlen(text));
        ufr_send(&link);
    }

    ufr_close(&link);
}

/*
  link:
  - empty
  - somente gateway
  - gateway, decoder
  - gateway, encoder

  estado:
  - reset
  - server, client, publisher, subscriber


*/

void test2() {
    link_t link;
    assert( ufr_new_gtw_posix_socket(&link, NULL) == LT_OK );
    assert( ufr_start_bind(&link, NULL) == LT_OK );
    assert( ufr_link_is_server(&link) == true );

    while(1) {
        link_t client;
        // client = lt_accept(&link);

        char* text = "HTTP/1.1 200\r\n"
            "Content-Type: text/plain\r\n"
            "\r\n"
            "OK e dai ?\n";

        lt_write(&client, text, strlen(text));
        lt_close(&client);
    }

    lt_close(&link);
}


int main() {
    test();
    return 0;
}