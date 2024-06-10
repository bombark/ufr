/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
	
// ============================================================================
//  Header
// ============================================================================

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

// ============================================================================
//  Functions
// ============================================================================

int get_file_free() {
    for (uint16_t i=0; i<FILE_TABLE_SIZE; i++) {
        if ( g_file_tab[i] == NULL ) {
            g_file_tab[i] = (void*) OCCUPIED;
            return i;
        }
    }
    return -1;
}

// ============================================================================
//  Main
// ============================================================================

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