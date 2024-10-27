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

// ============================================================================
//  Main
// ============================================================================

int main_complex() {
    // configure the output
    link_t link = ufr_sys_subscriber22("video1");

    while (1) {
        int num;
        ufr_get(&link, "^i", &num);
        printf("%d\n", num);
    }

    // end
    return 0;
}

int main_socket() {
    // configure the output
    link_t link = ufr_client("@new zmq:socket @coder msgpack @debug 4");

    // send command
    // for (int i=0; i<3; i++) {
        char command[1024];
        // scanf("%s", command);
        ufr_put(&link, "s\n\n", "list");

        // recv the answer
        while (1) {
            char buffer[1024];
            int v1,v2,v3;
            int res = ufr_get(&link, "^s", buffer);
            if ( res <= 0 ) {
                break;
            }
            printf("%s\n", buffer);
        }
    // }

    // end
    return 0;
}


int main_ros() {
    // configure the output
    link_t link = ufr_client("@new ros_melodic:socket");

    char buffer[1024];
    ufr_get(&link, "s", buffer);
    printf("%s\n", buffer);

    // end
    return 0;
}


#include <time.h>
#include <sys/time.h>

static
time_t get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    const time_t milliseconds = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    return milliseconds;
}


int main() {
    link_t timer = ufr_subscriber("@new posix:timer @time 250ms");
    link_t pose = ufr_subscriber("@new ros_humble:topic @msg pose @topic /turtle1/pose");
    // link_t motors = ufr_publisher("@new ros_humble:topic @msg twist @topic /turtle1/cmd_vel");

    for (int i=0; i<10; i++) {
        ufr_recv(&timer);
        uint32_t time;
        // ufr_get(&timer, "u", &time);

        printf("time\n");

        // update the robot pose
        float pos_x, pos_y;
        ufr_get(&pose, "^ff", &pos_x, &pos_y);
        
        printf("%f %f\n", pos_x, pos_y);
        // verifica a posicao do robo
        // calcula 
        
        // envia valores para os motores
        // ufr_put(&motors, "ffffff\n", 0.0, 0.0, 0.0, 0.0, 0.0, 0.2);
    }

    /*for (int i=0; i<5;) {
        if ( ufr_recv_async(&link) == 0 ) {
            time_t now = get_time_ms();
            printf("time %ld\n", now);
        }
    }*/

    return 0;
}










