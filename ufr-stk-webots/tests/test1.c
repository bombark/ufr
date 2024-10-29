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
#include <ufr.h>
#include <assert.h>
#include <unistd.h>
#include <math.h>

#include "ufr_webots.h"

// ============================================================================
//  Main
// ============================================================================

int main() {
    link_t motors;
    ufr_gtw_webots_new(&motors, 0);
    assert( ufr_boot_publisher(&motors, "@new webots @type motors") == UFR_OK );

    link_t encoders;
    ufr_gtw_webots_new(&encoders, 0);
    assert( ufr_boot_subscriber(&encoders, "@new webots @type encoders") == UFR_OK );

    link_t lidar;
    ufr_gtw_webots_new(&lidar, 0);
    assert( ufr_boot_subscriber(&lidar, "@new webots @type lidar") == UFR_OK );

    int count = 0;
    float lidar_values[500];
    float pos_x=0, pos_y=0, pos_th=0;
    float last_left=10, last_right=10;
    while ( ufr_loop_ok() ) {
        float left,right;
        ufr_get(&encoders, "^ff", &left, &right);
        if ( count == 0 ) {
            last_left = left;
            last_right = right;
            count = 1;
            continue;
        }
        

        const float diff_left = left - last_left;
        const float diff_right = right - last_right;
        

        pos_x += (diff_left + diff_right) * cos(pos_th);
        pos_y += (diff_left + diff_right) * sin(pos_th);
        pos_th += (diff_left - diff_right) * 0.3;

        printf("pose %f %f - %f %f %f\n", diff_left, diff_right, pos_x, pos_y, pos_th*180.0/3.141592);

        /* ufr_recv(&lidar);
        const size_t lidar_size = ufr_get_size(&lidar);
        for (size_t i=0; i<lidar_size; i++) {
            ufr_get(&lidar, "f", &lidar_values[i]);
            printf("%f ", lidar_values[i]);
        }
        printf("lidar %ld\n", lidar_size);*/

        ufr_put(&motors, "ff\n", -0.5, 0.0);

        last_left = left;
        last_right = right;
    }

    return 0;
}
