#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ufr.h>

int main(int argc, char** argv) {
    char command[128];
    while(1) {
        ufr_input("^s", command);
        printf("[LOG]: %s\n", command);
        if ( strcmp(command, "AT#EXIT") == 0 ) {
            ufr_output("s\n", "OK");
            break;
        } else if ( strcmp(command, "AT#PING") == 0 ) {
            ufr_output("s\n", "OK");
        } else {
            ufr_output("s\n", "ERROR");
        }
    }
    return 0;
}

