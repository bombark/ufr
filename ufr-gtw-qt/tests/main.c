#include <stdio.h>
#include <string.h>
#include <ufr.h>

int ufr_gtw_qt_new_socket(link_t* link, int type);
int ufr_gtw_qt_new_topic(link_t* link, int type);

int main() {
    link_t gtk;
    link_t event;

    ufr_gtw_qt_new_socket(&gtk, UFR_START_CLIENT);
    gtk.log_level = 0;
    ufr_args_t args = {.text=""};
    ufr_start_client(&gtk, &args);

    /*ufr_write(&gtk, "@create button @text botao", 27);
    ufr_recv(&gtk);

    ufr_gtk_new_topic(&event, UFR_START_SUBSCRIBER);
    event.log_level = 4;
    ufr_start_subscriber(&event, &args);

    */

   while ( ufr_loop_ok() ) {
        char id[1024];
        // ufr_get(&event, "^s", id);
        // printf("%s\n", id);
    }

    // ufr_close(&gtk);
    return 0;
}

/*

link_t fs = ufr_filesystem("@path ./");
link_t file = ufr_fs_open("fffff", "rw");



link_t dir = ufr_fs_open("diretory", "w");
ufr_put(&dir, "ss\n", "nome", "felipe");

node_t proc = {
    "gps", {.text="@new gps", .arg0=test}
}

node_t root = {
    "proc", {.text="@new directory", .arg0=&proc}
}




*/