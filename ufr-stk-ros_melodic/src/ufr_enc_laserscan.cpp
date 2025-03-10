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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ufr.h>
#include <sensor_msgs/LaserScan.h>

#include "ufr_gtw_ros_melodic.hpp"

struct Encoder {
    ros::Publisher publisher;
    sensor_msgs::LaserScan message;
    uint8_t index;
    uint16_t index2;

    Encoder() : index{0}, index2{0} {
    }

    void clear() {
        index = 0;
        index2 = 0;
    }
};

// ============================================================================
//  String Message Driver
// ============================================================================

static
int ufr_enc_ros_boot(link_t* link, const ufr_args_t* args) {
    Gateway* gtw_obj = (Gateway*) link->gtw_obj;
    Encoder* enc_obj = new Encoder();
    enc_obj->publisher = gtw_obj->node.advertise<sensor_msgs::LaserScan>("scan", 10);
    link->enc_obj = enc_obj;
    return UFR_OK;
}

static
void ufr_enc_ros_close(link_t* link) {
    if ( link->enc_obj ) {
        Encoder* enc = (Encoder*) link->enc_obj;
        delete enc;
    }
}

static
int ufr_enc_ros_put_u32(link_t* link, const uint32_t val[], int nitems) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    int i=0;
    for (; i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val[i]; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val[i]; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val[i]; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val[i]; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val[i]; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val[i]; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val[i]; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val[i];break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val[i];break;
            default: break;
        }
    }
    return i;
}

static
int ufr_enc_ros_put_i32(link_t* link, const int32_t val[], int nitems) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    int i=0;
    for (; i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val[i]; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val[i]; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val[i]; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val[i]; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val[i]; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val[i]; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val[i]; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val[i];break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val[i];break;
            default: break;
        }
    }
    return i;
}

static
int ufr_enc_ros_put_f32(link_t* link, const float val[], int nitems) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    int i=0;
    for (; i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val[i]; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val[i]; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val[i]; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val[i]; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val[i]; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val[i]; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val[i]; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val[i];break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val[i];break;
            default: break;
        }
    }
    return i;
}

static
int ufr_enc_ros_put_str(link_t* link, const char* val) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {

    }
    return 0;
}

static
int ufr_enc_ros_put_cmd(link_t* link, char cmd) {
    static int seq_id = 0;

    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( cmd == '\n' ) {
        enc_obj->message.header.seq = seq_id++;
        enc_obj->message.header.stamp = ros::Time::now();
        enc_obj->message.header.frame_id = "laser_frame";

// printf("laserstamp %f %f\n", enc_obj->message.ranges[10], enc_obj->message.intensities[10]);
        enc_obj->publisher.publish(enc_obj->message);
        // enc_obj->message.clear();
        enc_obj->clear();
        ufr_log(link, "sent laserscan message");
    }
    return 0;
}

static
int ufr_ros_enter(struct _link* link, size_t maxsize) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj->index == 7 ) {
        enc_obj->message.ranges.resize(maxsize);
        enc_obj->index2 = 0;
    } else if ( enc_obj->index == 8 ) {
        enc_obj->message.intensities.resize(maxsize);
        enc_obj->index2 = 0;
    } else {
        printf("errooo de index %d\n", enc_obj->index);
        return 1;
    }
    return UFR_OK;
}

static
int ufr_ros_leave(struct _link* link) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    enc_obj->index += 1;
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_ros = {
    .boot = ufr_enc_ros_boot,
    .close = ufr_enc_ros_close,
    .clear = NULL,

    .put_u32 = ufr_enc_ros_put_u32,
    .put_i32 = ufr_enc_ros_put_i32,
    .put_f32 = ufr_enc_ros_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_enc_ros_put_cmd,
    .put_str = ufr_enc_ros_put_str,
    .put_raw = NULL,

    .enter = ufr_ros_enter,
    .leave = ufr_ros_leave,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros_melodic_new_laserscan(link_t* link, const int type) {
    link->enc_api = &ufr_enc_ros;
    return 0;
}
