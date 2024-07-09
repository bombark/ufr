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
#include <cwalk.h>

#include <sys/stat.h>

#define FILE_TABLE_SIZE 32

#define FREE      0
#define OCCUPIED  1

FILE* g_file_tab[FILE_TABLE_SIZE];

char current_path[4096];

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

int builtin_mkdir(link_t* server) {
    int retcode = -1;
    char* retmesg = "";

    char arg1[1024];
    if ( ufr_get(server, "s", arg1) == 1 ) {
        char dir_path[1024];
        cwk_path_join(current_path, arg1, dir_path, 1024);
        mkdir(dir_path, 0755);
        retcode = UFR_OK;
        retmesg = "OK";
    } else {
        retcode = -1;
        retmesg = "ERROR";
    }

    ufr_get_eof(server);
    ufr_put(server, "is\n", retcode, retmesg);
    ufr_put_eof(server);

    return retcode;
}

int builtin_ls(link_t* server) {
    int retcode;
    ufr_get_eof(server);
    DIR *dp = opendir (current_path);
    struct dirent *ep;
    if ( dp != NULL ) {
        retcode = UFR_OK;
        ufr_put(server, "is\n", retcode, "OK");
        while ((ep = readdir (dp)) != NULL) {
            ufr_put(server, "s\n", ep->d_name);
        }
        closedir(dp);
    } else {
        ufr_put(server, "is\n", -1, "ERROR");
    }
    ufr_put_eof(server);
    return retcode;
}


// ============================================================================
//  Main
// ============================================================================

int main() {
    // configure the output
    link_t server = ufr_server("@new zmq:socket @coder msgpack @debug 4");
    strcpy(current_path, "./data");

    // publish 5 messages
    for (int i=0; i<10; i++) {
        // recv
        char command[1024];
        ufr_get(&server, "^s", command);


        if ( strcmp(command, "cd") == 0 ) {
            char arg1[1024];
            if ( ufr_get(&server, "s", arg1) == 1 ) {
                cwk_path_join(current_path, arg1, current_path, 1024);
            } else {
                strcpy(current_path, "/");
            }
            ufr_get_eof(&server);

            ufr_put(&server, "s\n", current_path);
            ufr_put_eof(&server);

        // start
        } else if ( strcmp(command, "start") == 0 ) {
            ufr_get_eof(&server);
            ufr_put(&server, "i\n", UFR_OK);
            ufr_put_eof(&server);

        // ls
        } else if ( strcmp(command, "ls") == 0 ) {
            builtin_ls(&server);

        // mkdir
        } else if ( strcmp(command, "mkdir") == 0 ) {
            builtin_mkdir(&server);

        } else {
            ufr_get_eof(&server);
            printf("ERROR %s\n", command);
            ufr_put(&server, "s\n", "ERROR");
            ufr_put_eof(&server);
        }
       
    }

    // end
    ufr_close(&server);
    return 0;
}