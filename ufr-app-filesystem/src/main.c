#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <ufr.h>

#include <sys/stat.h>

#define FILE_TABLE_SIZE 32

#define FREE      0
#define OCCUPIED  1

FILE* g_file_tab[FILE_TABLE_SIZE];

int get_file_free() {
    for (uint16_t i=0; i<FILE_TABLE_SIZE; i++) {
        if ( g_file_tab[i] == NULL ) {
            g_file_tab[i] = (void*) OCCUPIED;
            return i;
        }
    }
    return -1;
}


int main() {
    ufr_inoutput_init("@new zmq:socket @port 4000 @coder msgpack:obj");

    for (uint16_t i=0; i<FILE_TABLE_SIZE; i++) {
        g_file_tab[i] = NULL;
    }

    char command[512];
    while(1) {
        ufr_input("^s", command);

        // ping service
        if ( strcmp(command, "ping") == 0 ) {
            ufr_output("s\n", "OK");

        // read whole file
        } else if ( strcmp(command, "open") == 0 ) {
            char name[512];
            char mode[512];
            ufr_input("ss", name, mode);
            int pos = get_file_free();
            if ( pos < 0 ) {
                ufr_output("s\n", "ERROR");
            }
            FILE* fd = fopen(name, mode);
            if ( fd == NULL ) {
                ufr_output("s\n", "ERROR");
            } else {
                g_file_tab[pos] = fd;
                ufr_output("is\n", pos, "OK");
            }

        // close file description
        } else if ( strcmp(command, "close") == 0 ) {
            uint16_t fd_pos;
            ufr_input("i", &fd_pos);
            if ( fd_pos >= FILE_TABLE_SIZE ) {
                ufr_output("s\n", "ERROR");
                continue;
            }

            FILE* fd = g_file_tab[fd_pos];
            if ( fd ) {
                fclose(fd);
            }
            g_file_tab[fd_pos] = NULL;
            ufr_output("s\n", "OK");

        // write whole file
        } else if ( strcmp(command, "write") == 0 ) {
            uint16_t fd_pos;
            ufr_input("i", &fd_pos);
            if ( fd_pos >= FILE_TABLE_SIZE ) {
                ufr_output("s\n", "ERROR");
                continue;
            }
            FILE* fd = g_file_tab[fd_pos];
            if ( fd == NULL ) {
                ufr_output("s\n", "ERROR");
                continue;
            }
            char buffer[512];
            ufr_input("s", buffer);
            fwrite(buffer, strlen(buffer), 1, fd);

        // list directory
        } else if ( strcmp(command, "ls") == 0 || strcmp(command, "list") == 0 ) {
            DIR *dp = opendir ("./");
            if (dp != NULL) {
                struct dirent *ep;
                while ((ep = readdir(dp)) != NULL) {
                    ufr_output("s", ep->d_name);
                }
                ufr_output("\n");
                closedir(dp);
            } else {
                ufr_output("s\n", "ERROR");
            }

        // mkdir directory
        } else if ( strcmp(command, "mkdir") == 0 ) {
            char name[512];
            ufr_input("s", name);
            int rt = mkdir(name, 0755);
            if ( rt == 0 ){
                ufr_output("s\n", "OK");
            } else {
                ufr_output("s\n", "ERROR");
            }
        
        // create file
        } else if ( strcmp(command, "touch") == 0 ) {
            char name[512];
            ufr_input("s", name);
            FILE* fd = fopen(name, "w");
            fclose(fd);
        
        // exit
        } else if ( strcmp(command, "exit") == 0 ) {
            break;

        // invalid command 
        } else {
            ufr_output("s\n", "ERROR");
        }
    }


    return 0;
}