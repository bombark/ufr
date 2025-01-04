#include <stdio.h>
#include <ufr.h>

int ufr_gtw_http_socket_new(link_t* link, int type);


int main() {
    link_t server;
    ufr_gtw_http_socket_new(&server, NULL);
    ufr_boot_gtw(&server, NULL);

    while(1) {
    }

    ufr_close(&server);
    return 0;
}