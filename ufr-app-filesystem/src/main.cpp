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

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <ufr.h>

using namespace cv;

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
int16_t encoder_left_val;
int16_t encoder_right_val;

// ============================================================================
//  Main
// ============================================================================

int main() {
    // configure the output
    // link_t server = ufr_server_st("@new zmq:socket @coder msgpack @debug 0");
    // link_t left = ufr_subscriber("@new ros_melodic:topic @msg i16 @topic left_encoder @debug 0");
    // link_t right = ufr_subscriber("@new ros_melodic:topic @msg i16 @topic right_encoder @debug 4");

    link_t server = ufr_server_st("@new zmq:socket @coder msgpack");
    link_t motors = ufr_publisher("@new webots @type motors");   
    link_t encoders = ufr_subscriber("@new webots @type encoders");
    link_t lidar = ufr_subscriber("@new webots @type lidar");

    float left=0, right=0;
    float pos_x=0, pos_y=0, pos_th=0;
    float lidar_ranges[2000];

    Mat map(320, 320, CV_8UC1);

    // get first read from encoder
    ufr_loop_ok();
    ufr_get(&encoders, "^ff", &left, &right);    

    while ( ufr_loop_ok() ) {
        const float last_left = left;
        const float last_right = right;       

        // int topic_id = ufr_recv_2a(&server, &encoders, 100);
        ufr_get(&encoders, "^ff", &left, &right);

        // calculate de position
        const float diff_left = left - last_left;
        const float diff_right = right - last_right;
        pos_x += (diff_left + diff_right) * cos(pos_th);
        pos_y += (diff_left + diff_right) * sin(pos_th);
        pos_th += (diff_left - diff_right) * 0.3;

        // printf("%f %f %f\n", pos_x, pos_y, pos_th);
        // ufr_put(&motors, "ff\n", 0.0, 1.0);

        ufr_recv(&lidar);
        size_t lidar_size = ufr_get_size(&lidar);
        ufr_get_af32(&lidar, lidar_ranges, lidar_size);

        if ( ufr_recv_async(&server) == UFR_OK ) {
            char command[1024];
            ufr_get(&server, "s", command);
            printf("server %s ", command);

            if ( strcmp(command, "odom") == 0 ) {
                ufr_get_eof(&server);
                ufr_put(&server, "fff\n", pos_x, pos_y, pos_th);
                ufr_put_eof(&server);
                printf("- OK\n");

            } else if ( strcmp(command, "scan") == 0 ) {
                ufr_get_eof(&server);
                ufr_put(&server, "af\n", lidar_size, lidar_ranges);
                ufr_put(&server, "\n");
                printf("%ld - OK\n", lidar_size);

            } else if ( strcmp(command, "scan_map") == 0 ) {
                ufr_get_eof(&server);
                map = 0;
                map.at<uint8_t>(160,160) = 200; 

                float angle = M_PI/2.0;
                float delta_angle = M_PI*2.0 / lidar_size;
                for (int i=0; i<lidar_size; i++, angle+=delta_angle) {
                    const float dist = lidar_ranges[i];
                    if ( dist == INFINITY ) {
                        continue;
                    }

                    uint32_t y = 160 - ( dist * sin(angle) * 25.0 );
                    uint32_t x = 160 + ( dist * cos(angle) * 25.0 );
                    if ( x < 320 && y < 320 ) {
                        map.at<uint8_t>(y,x) = 255;
                    }
                }

                std::vector<uchar> buf;
                imencode(".jpg", map, buf);
                ufr_put_raw(&server, buf.data(), buf.size());
                ufr_put(&server, "\n");
                ufr_put_eof(&server);
                printf(" - OK\n");

            } else if ( strcmp(command, "cmd_vel") == 0 ) {
                float vel, rotvel;
                ufr_get(&server, "ff\n", &vel, &rotvel);
                printf("%f %f\n", vel, rotvel);
                ufr_put(&motors, "ff\n", vel, rotvel);

                ufr_put(&server, "is\n", 0, "OK");
                ufr_put_eof(&server);

            } else {
                printf(" - ERROR\n");
            }

        }

        // ufr_put(&motors, "ff\n", 0.0, 1.0);

        /* int topic_id = ufr_recv_2a(&server, &encoders, 100);
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

            } else if ( strcmp(command, "encoder") == 0 ) {
                ufr_put(&server, "ii\n\n", encoder_right_val, encoder_left_val);

            } else if ( strcmp(command, "image") == 0 ) {
                ufr_put_raw(&server, g_image_data, g_image_size);
                ufr_put(&server, "\n");
                ufr_put_eof(&server);

            } else if ( strcmp(command, "scan") == 0 ) {
                // ufr_put(&server, "iii\n\n", 10, 20, 30);
                ufr_put_af32(&server, g_lidar.ranges, 200);
                ufr_put(&server, "\n");
                ufr_put_eof(&server);

            } else if ( strcmp(command, "list") == 0 ) {
                ufr_put(&server, "ss\n", "odom", "iii");
                ufr_put(&server, "ss\n", "image", "siiiab");
                ufr_put(&server, "ss\n", "scan", "af");
                ufr_put_eof(&server);
            }

        } else if ( topic_id == 1 ) {
            ufr_get(&encoders, "ff", &left, &right);
            printf("%f %f\n", left, right);

        } else if ( topic_id == 2 ) {


        } else {
            printf("erro %d\n", topic_id);
        }*/
    }

    // end
    // ufr_close(&server);
    return 0;
}
