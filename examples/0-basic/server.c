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

uint8_t g_image_data[1024*1024];
size_t g_image_size = 0;

// ============================================================================
//  Main
// ============================================================================

int main() {
    // configure the output
    link_t server = ufr_server_st("@new zmq:socket @coder msgpack @debug 4");
    link_t image = ufr_subscriber("@new zmq:topic @coder msgpack @port 3001 @debug 4");

    for (int i=0; i<1000; i++) {
        char command[1024];
        int topic_id = ufr_recv_2a(&server, &image, 100);
        if ( topic_id < 0 ) {
            continue;
        }
        printf("topic %d\n", topic_id);

        if ( topic_id == 0 ) {
            ufr_get(&server, "s\n", command);
            printf("[LOG]: %s\n", command);

            if ( strcmp(command, "odom") == 0 ) {
                ufr_put(&server, "iii\n\n", 10, 20, 30);

            } else if ( strcmp(command, "image") == 0 ) {
                ufr_put_raw(&server, g_image_data, g_image_size);
                ufr_put(&server, "\n");
                ufr_put_eof(&server);

            } else if ( strcmp(command, "scan") == 0 ) {
                ufr_put(&server, "iii\n\n", 10, 20, 30);

            } else if ( strcmp(command, "list") == 0 ) {
                ufr_put(&server, "ss\n", "odom", "iii");
                ufr_put(&server, "ss\n", "image", "siiiab");
                ufr_put(&server, "ss\n", "scan", "af");
                ufr_put_eof(&server);
            }
        } else if ( topic_id == 1 ) {
            int rows, cols;
            ufr_get(&image, "ii", &rows, &cols);
            printf("%d %d\n", rows, cols);
            const size_t image_size = ufr_get_size(&image); 
            const char* image_ptr = ufr_get_raw_ptr(&image);
            memcpy(g_image_data, image_ptr, image_size);
            g_image_size = image_size;
            printf("tamanho %d\n", g_image_size);

        } else {
            printf("erro %d\n", topic_id);
        }

    }

    // end
    ufr_close(&server);
    return 0;
}
