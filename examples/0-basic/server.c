/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
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
#include <string.h>
#include <ufr.h>
#include <sys/types.h>
#include <dirent.h>
#include <cwalk.h>

typedef struct {
    char name[64];
} node_t;

typedef struct {
    uint8_t size;
    node_t nodes[64];
} directory_t;

directory_t g_root;

// ============================================================================
//  Directory Class Functions
// ============================================================================

void directory_init(directory_t* dir) {
    dir->size = 0;
}

void directory_add_node(directory_t* dir, const char* name) {
    const uint8_t pos = dir->size;
    dir->size += 1;
    strcpy(&dir->nodes[pos], name);
}

// ============================================================================
//  Functions
// ============================================================================



// ============================================================================
//  Main
// ============================================================================

int main() {
    directory_init(&g_root);


    // configure the output
    link_t server = ufr_server("@new zmq:socket @coder msgpack @debug 4");
    char current_path[1024];
    strcpy(current_path, "./aaa");

    // publish 5 messages
    for (int i=0; i<10; i++) {
        // recv
        char command[1024];
        ufr_get(&server, "^s", command);


        if ( strcmp(command, "cd") == 0 ) {
            char arg1[1024];
            int a = ufr_get(&server, "s", arg1);
            printf("l %d\n", a);
            printf("%s %s\n", current_path, arg1);
            char buffer[1024];
            cwk_path_join(current_path, arg1, buffer, 1024);
            printf("aaa %s\n", buffer);
            ufr_get(&server, "^");

            ufr_put(&server, "s\n", current_path);
            ufr_put_eof(&server);

        } else if ( strcmp(command, "ls") == 0 ) {
            ufr_get(&server, "^");

            DIR *dp = opendir (current_path);
            struct dirent *ep;     
            if ( dp != NULL ) {
                while ((ep = readdir (dp)) != NULL) {
                    ufr_put(&server, "s\n", ep->d_name);
                }
                closedir(dp);
            }
            ufr_put_eof(&server);

        } else {
            ufr_get(&server, "^");
            printf("%s\n", command);
            ufr_put(&server, "s\n", "ERROR");
            ufr_put_eof(&server);
        }
       
    }

    // end
    ufr_close(&server);
    return 0;
}


/*
int main() {
    // configure the output
    link_t server = ufr_server("@new zmq:socket @coder msgpack @debug 4");

    // publish 5 messages
    for (int i=0; i<5; i++) {
        // recv
        int command;
        ufr_get(&server, "^i", &command);
        ufr_get(&server, "^");

        // send
        ufr_put(&server, "ii\n", 51,61);
        ufr_put(&server, "ii\n", 52,62);
        ufr_put(&server, "ii\n", 53,63);
        ufr_put_eof(&server);
    }

    // end
    return 0;
}
*/