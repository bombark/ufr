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
#include <opencv2/opencv.hpp>

uint8_t g_image_data[1024*1024];
size_t g_image_size = 0;

typedef struct {
    float angle_min, angle_max, angle_increment;
    float time_increment, scan_time;
    float range_min, range_max;
    size_t ranges_len;
    float ranges[1200];
    size_t intensities_len;
    float intensities[1200];
} lidar_t;

lidar_t g_lidar;

using namespace cv;
using namespace std;

// ============================================================================
//  Main
// ============================================================================

int main() {
    // configure the output
    link_t server = ufr_server_st("@new zmq:socket @coder msgpack @debug 4");
    // link_t scan = ufr_subscriber("@new ros_humble:topic @msg laserscan @topic scan");
    link_t scan = ufr_subscriber("@new zmq:topic @coder msgpack @port 3001");

    Mat scan_mat(480, 640, CV_8UC1);
    vector<uchar> scan_jpg;

    for (int i=0; i<5000; i++) {
        char command[1024];
        int topic_id = ufr_recv_2a(&server, &scan, 100);
        if ( topic_id < 0 ) {
            continue;
        }
        // printf("topic %d\n", topic_id);
        // ufr_recv(&server);
        //int topic_id = 0;

        if ( topic_id == 0 ) {
            ufr_get(&server, "s\n", command);
            printf("[LOG]: %s\n", command);

            if ( strcmp(command, "odom") == 0 ) {
                ufr_put(&server, "iii\n\n", 10, 20, 30);

            } else if ( strcmp(command, "image") == 0 ) {
                FILE* fd = fopen("teste.png", "r");
                g_image_size = fread(g_image_data, 1, 1024*1024, fd);
                fclose(fd);

                ufr_put_raw(&server, g_image_data, g_image_size);
                ufr_put(&server, "\n");
                ufr_put_eof(&server);

            } else if ( strcmp(command, "scan") == 0 ) {
                
                /*imencode(".jpg", scan_mat, scan_jpg);
printf("%d\n", scan_jpg.size());          
                ufr_put_raw(&server, scan_jpg.data(), scan_jpg.size());
                ufr_put(&server, "\n");
                ufr_put_eof(&server);*/

                // ufr_put(&server, "iii\n\n", 10, 20, 30);
                // ufr_put_af32(&server, g_lidar.ranges, 200);
                ufr_put(&server, "\n");
                ufr_put_eof(&server);

            } else if ( strcmp(command, "list") == 0 ) {
                ufr_put(&server, "ss\n", "odom", "iii");
                ufr_put(&server, "ss\n", "image", "siiiab");
                ufr_put(&server, "ss\n", "scan", "af");
                ufr_put_eof(&server);
            }

        } else if ( topic_id == 1 ) {
            ufr_get(&scan, "fff", &g_lidar.angle_min, &g_lidar.angle_max, &g_lidar.angle_increment);
            // ufr_get(&scan, "ff", &g_lidar.time_increment, &g_lidar.scan_time);
            // ufr_get(&scan, "ff", &g_lidar.range_min, &g_lidar.range_max);
            ufr_get(&scan, "af", g_lidar.ranges, 1200);
            ufr_get(&scan, "af", g_lidar.intensities, 1200);
            printf("%f %f\n", g_lidar.angle_min, g_lidar.angle_max);

        } else if ( topic_id == 3 ) {
            /* int rows, cols;
            ufr_get(&image, "ii", &rows, &cols);
            printf("%d %d\n", rows, cols);
            const size_t image_size = ufr_get_size(&image); 
            const char* image_ptr = ufr_get_raw_ptr(&image);
            memcpy(g_image_data, image_ptr, image_size);
            g_image_size = image_size;
            printf("tamanho %d\n", g_image_size);*/

        } else {
            printf("erro %d\n", topic_id);
        }
    }

    // end
    ufr_close(&server);
    return 0;
}
