#include <stdio.h>
#include <lt_api.h>

int lt_new_httpd_socket(link_t* link, const lt_args_t* args);


int main() {
    link_t server;
    lt_new_httpd_socket(&server, NULL);
    lt_start_bind(&server);

    while(1) {
        lt_recv(&server);
        lt_write(&server, "Opa!", 4);
    }

    lt_close(&server);
    return 0;
}