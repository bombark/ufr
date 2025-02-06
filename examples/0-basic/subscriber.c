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
#include <ufr.h>
#include <math.h>

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
// const double PI = 3.141592;
 
// Robot physical constants
const double TICKS_PER_REVOLUTION = 620; // For reference purposes.
const double WHEEL_RADIUS = 0.033; // Wheel radius in meters
const double WHEEL_BASE = 0.22; // Center of left tire to center of right tire
const double TICKS_PER_METER = 2800; // Original was 2800


typedef struct {
    float x,y,th;
} Robot;

Robot g_robot;


// ============================================================================
//  Functions
// ============================================================================

void calculate_position_from_encoder(const int16_t left, const int16_t right) {
    // Static variable to save the last count from encoder
    static uint8_t is_first_call = 1;
    static int16_t last_left;
    static int16_t last_right;
    static uint16_t call = 0;

    /*call += 1;
    if ( call > 2010 ) {
        exit(0);
    }*/

    // First Call
    if ( is_first_call ) {
        last_left = left;
        last_right = right;
        is_first_call = 0;
        return;
    }

    int left_ticks = left - last_left;
    if ( abs(left_ticks) > 50 ) {
        left_ticks = 0;
    }

    int right_ticks = right - last_right;
    if ( abs(right_ticks) > 50 ) {
        right_ticks = 0;
    }

    //
    const double left_distance = (2.0 * M_PI * WHEEL_RADIUS * left_ticks) / TICKS_PER_REVOLUTION;
    const double right_distance = (2.0 * M_PI * WHEEL_RADIUS * right_ticks) / TICKS_PER_REVOLUTION;
    const double delta_distance = (left_distance + right_distance) / 2.0;
    const double delta_theta = (right_distance - left_distance) / WHEEL_BASE;

    // printf("%d\n", call);
    // printf("a %d %d - (%d %d) -> %d %d\n", left, right, last_left, last_right, left_ticks, right_ticks);
    // printf("b %f %f %f\n", left_distance, right_distance, delta_theta);

    g_robot.x += cos(g_robot.th) * delta_distance;
    g_robot.y += sin(g_robot.th) * delta_distance;
    g_robot.th += delta_theta;

    // printf("c %f %f %f\n", g_robot.x, g_robot.y, g_robot.th*180.0/M_PI);
    // printf("\n");

    // Save the last count from encoder
    last_left = left;
    last_right = right;
}


// ============================================================================
//  Main
// ============================================================================

int main() {
    g_robot.x = 0;
    g_robot.y = 0;
    g_robot.th = 0;

    link_t left_sub = ufr_subscriber("@new ros_melodic:topic @msg i16 @topic left_encoder");
    link_t right_sub = ufr_subscriber("@new ros_melodic:topic @msg i16 @topic right_encoder");


    // link_t timer = ufr_subscriber("@new posix:timer @time 1s");
    link_t odom_pub = ufr_publisher("@new ros_melodic:topic @msg pose");

    /*
     - Problema do ufr_loop_ok nao estar saindo
     - Fazer um esquema para sair do laço ufr_loop_ok
     - pensar em como fazer um header para a mensagem
    */

    // ufr_header(&left_sub, "data");
    // ufr_header(&right_sub, "data");

    while( ufr_loop_ok() ) {
        int left, right;

        if ( ufr_recv_2s(&left_sub, &right_sub, 50) == UFR_OK ) {
            ufr_get(&left_sub, "i", &left);
            ufr_get(&right_sub, "i", &right);
            calculate_position_from_encoder(left, right);
            ufr_put(&odom_pub, "fff\n", g_robot.x, g_robot.y, g_robot.th);
        } 

        /*if ( ufr_recv(&timer) == UFR_OK ) {
            printf("enviando\n");
            ufr_put(&odom_pub, "fff\n", g_robot.x, g_robot.y, g_robot.th);
            g_robot.x += 0.125;
        }*/
    }

    // end
    ufr_close(&left_sub);
    ufr_close(&right_sub);
    return 0;
}
