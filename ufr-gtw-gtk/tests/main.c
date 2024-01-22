#include <stdio.h>
#include <string.h>
#include <ufr.h>

int lt_new_gtk_socket(link_t* link, const lt_args_t* args);

int main() {
    link_t link;
    lt_new_gtk_socket(&link, NULL);

    char* text = "Opa!";
    lt_write(&link, text, strlen(text));

    lt_close(&link);
    return 0;
}