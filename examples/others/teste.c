#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <ufr.h>

link_t g_ctrl;

bool ufr_ctrl_is_ok() {
    char buffer[512];
    lt_put(&g_ctrl, "s\n", "ping");
    lt_get(&g_ctrl, "^s", buffer);
    if ( strncmp(buffer, "OK", 2) == 0 ) {
        return true;
    }
    return false;
}


int main() {
    g_ctrl = ufr_new("@new zmq:socket @port 4000 @coder msgpack:obj");
    lt_start_connect(&g_ctrl);

    while(1) {
        char line[512];
        printf("root# ");
        fgets(line, 512, stdin);
        if ( strlen(line) == 1 ) {
            continue;
        }

        printf("%d %s\n", strlen(line), line);
        lt_put(&g_ctrl, "s\n", line);
        if ( lt_recv(&g_ctrl) ) {
            char nome[512];
            while ( ufr_get_str(&g_ctrl, nome) ) { 
                printf("%s\n", nome);
            }
        }
        
    }

    lt_close(&g_ctrl);
    return 0;
}