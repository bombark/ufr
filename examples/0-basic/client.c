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
#include <cwalk.h>

link_t g_link;
char g_current_path[1024];

// ============================================================================
//  Functions
// ============================================================================

int builtin_cd(const char* path) {
    // Send the command to the server
    if ( path == NULL ) {
        ufr_put(&g_link, "s\n", "cd");
    } else {
        ufr_put(&g_link, "ss\n", "cd", path);
    }
    ufr_put_eof(&g_link);

    // Receive the answer
    char response[1024];
    int res = ufr_get(&g_link, "^s", response);
    printf("%s\n", response);

    // Update the global variable: current path
    strcpy(g_current_path, response);

    // Success
    return UFR_OK;
}


// ============================================================================
//  Main
// ============================================================================

int main() {
    strcpy(g_current_path, "./");
    g_link = ufr_client("@new zmq:socket @coder msgpack @debug 0");

    printf("root:%s$ ", g_current_path);
    while(1) {
        char line[1024];
        fgets(line, 1024, stdin);

        const char* command = strtok(line, " \n");
        if ( command != NULL ) {
            if ( strcmp(command, "q") == 0 || strcmp(command, "quit") == 0 ) {
                break;

            // other commands
            } else if ( strcmp(command, "cd") == 0 ) {
                const char* path = strtok(NULL, " \n");
                builtin_cd(path);

            } else {
                // send the command
                ufr_put(&g_link, "s\n", command);
                ufr_put_eof(&g_link);

                // wait for the response
                while ( 1 ) {
                    char response[1024];
                    int res = ufr_get(&g_link, "^s", response);
                    if ( res <= 0 ) {
                        break;
                    }
                    printf("%s\n", response);
                }
            }
        }

        printf("root:%s$ ", g_current_path);
    }

    // end
    ufr_close(&g_link);
    return 0;
}


int main222() {
    // configure the output
    link_t link = ufr_client("@new zmq:socket @coder msgpack @debug 4");

    {
        ufr_put(&link, "ss\n", "cd", "include");
        ufr_put_eof(&link);
        char response[1024];   
        ufr_get(&link, "^s", response);
        ufr_get(&link, "^");
    }
    
    {
        // send command
        ufr_put(&link, "s\n", "ls");
        ufr_put_eof(&link);

        // recv the answer
        while ( 1 ) {
            char response[1024];
            int res = ufr_get(&link, "^s", response);
            if ( res == 0 ) {
                break;
            }
            printf("%s\n", response);
        }
    }

    // end
    return 0;
}

/*
int main() {
    // configure the output
    link_t link = ufr_client("@new zmq:socket @coder msgpack @debug 4");

    // send command
    ufr_put(&link, "i\n", 10);
    ufr_put_eof(&link);

    // recv the answer
    int v1,v2,v3;
    ufr_get(&link, "^ii", &v1, &v2);
    printf("%d %d\n", v1, v2);
    ufr_get(&link, "^ii", &v1, &v2);
    printf("%d %d\n", v1, v2);
    ufr_get(&link, "^ii", &v1, &v2);
    printf("%d %d\n", v1, v2);
    ufr_get(&link, "^");

    // end
    return 0;
}q
*/

// sexta, 17:00 -> 18:00 workshop pioneers
// explicar os comandos basicos do github
// explicar sobre cmake
// ssh

