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
#include <stdlib.h>
#include <string.h>
#include <ufr.h>
#include <errno.h>

// ============================================================================
//  Main
// ============================================================================

int main() {
    // configure the output
    link_t server = ufr_server_st("@new zmq:socket @host 127.0.0.1 @port 3000 @coder msgpack @debug 4");

    // publish 5 messages
    while(1) {
        char command[1024];
        ufr_get(&server, "^s", command);
        printf("LOG: %s\n", command);
        if ( strcmp(command, "open") == 0 ) {
            char path[1024];
            ufr_get(&server, "s", path);
            FILE* fd = fopen(path,"r");
            if ( fd ) {
                char buffer[1024];
                if ( fgets(buffer, 1024, fd) != NULL ) {
                    ufr_put(&server, "is\n", 0, buffer);
                } else {
                    ufr_put(&server, "is\n", 1, "");
                }
                fclose(fd);
            } else {
                ufr_put(&server, "is\n", 1, strerror(errno));
            }

        } else if ( strcmp(command, "make") == 0 ) {
            char name[1024];
            char link_params[4096];
            ufr_get(&server, "ss", name, link_params);

            char ip[1024];
            ufr_recv_peer_name(&server, ip, 1024);
            strcat(link_params, " @host ");
            strcat(link_params, ip);

            FILE* fd = fopen(name, "w");
            if ( fd ) {
                fprintf(fd, "%s\n", link_params);
                fclose(fd);
                ufr_put(&server, "is\n", 0, link_params);
            } else {
                ufr_put(&server, "is\n", 1, "ERROR");
            }

        } else if ( strcmp(command, "quit") == 0 ) {
            break;

        } else {
            printf("error\n");
        }
    }

    // end
    return 0;
}

