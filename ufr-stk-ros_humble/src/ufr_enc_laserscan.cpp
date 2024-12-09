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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ufr_gtw_ros_humble.hpp"

struct ll_encoder {
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
    sensor_msgs::msg::LaserScan message;
    int index;
    int index2;
    int frame_id;

    float def_angle_min;
    float def_angle_max;

    ll_encoder() : index{0}, index2{0}, frame_id{0} {}
};

// ============================================================================
//  LaserScan Encoder
// ============================================================================

static
int ufr_enc_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    std::string topic_name = ufr_args_gets(args, "@topic", "topico");

    ll_encoder* enc_obj = new ll_encoder();
    // enc_obj->def_angle_max = ufr_args_getf(args, "@angle_max", 0.0);
    // @order angle_max,angle_min,ranges @default {range_min: 0, range_max: 10}

    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    enc_obj->publisher = gtw_obj->m_node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
    link->enc_obj = enc_obj;
    ufr_info(link, "loaded encoder for sensor_msgs/LaserScan");

    return UFR_OK;
}

static
int ufr_enc_ros_humble_put_u32(link_t* link, uint32_t val) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val;break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val;break;
            default: break;
        }
    }
    return 0;
}

static
int ufr_enc_ros_humble_put_i32(link_t* link, int32_t val) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val;break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val;break;
            default: break;
        }
    }
    return 0;
}

static
int ufr_enc_ros_humble_put_f32(link_t* link, float val) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val;break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val;break;
            default: break;
        }
    }
    return 0;
}

static
int ufr_enc_ros_humble_put_str(link_t* link, const char* val) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj ) {

    }
    return 0;
}

static
int ufr_enc_ros_humble_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj->index == 7 ) {
        if ( type == 'i' ) {
            const int32_t* src = (const int32_t*) arr_ptr;
            enc_obj->message.ranges.resize(arr_size);
            for (size_t i=0; i<arr_size; i++) {
                enc_obj->message.ranges[i] = src[i];
            }
        } else if ( type == 'f' ) {
            const float* src = (const float*) arr_ptr;
            enc_obj->message.ranges.resize(arr_size);
            for (size_t i=0; i<arr_size; i++) {
                enc_obj->message.ranges[i] = src[i];
            }
        }
    } else if ( enc_obj->index == 8 ) {
        if ( type == 'i' ) {
            const int32_t* src = (const int32_t*) arr_ptr;
            enc_obj->message.intensities.resize(arr_size);
            for (size_t i=0; i<arr_size; i++) {
                enc_obj->message.intensities[i] = src[i];
            }
        } else if ( type == 'f' ) {
            const float* src = (const float*) arr_ptr;
            enc_obj->message.intensities.resize(arr_size);
            for (size_t i=0; i<arr_size; i++) {
                enc_obj->message.intensities[i] = src[i];
            }
        }
    } else {
        return 1;
    }
    return 0;
}

static
int ufr_ecr_ros_humble_put_cmd(link_t* link, char cmd) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( cmd == '\n' ) {
        enc_obj->publisher->publish(enc_obj->message);
        enc_obj->index = 0;
        enc_obj->frame_id += 1;
        // enc_obj->message.header.stamp = rclcpp::Time.now();
        enc_obj->message.header.frame_id = std::to_string(enc_obj->frame_id);
        ufr_info(link, "sent message sensors/LaserScan");
    }
    return 0;
}

static
int ufr_ros_topic_enter_array(struct _link* link, size_t maxsize) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj->index == 7 ) {
        enc_obj->message.ranges.resize(maxsize);
        enc_obj->index2 = 0;
    } else if ( enc_obj->index == 8 ) {
        enc_obj->message.intensities.resize(maxsize);
        enc_obj->index2 = 0;
    } else {
        printf("errooo\n");
        return 1;
    }
    return UFR_OK;
}

static
int ufr_ros_topic_leave_array(struct _link* link) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    enc_obj->index += 1;
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_ros_api = {
    .boot = ufr_enc_ros_humble_boot,
    .close = NULL,
    .clear = NULL,

    .put_u32 = ufr_enc_ros_humble_put_u32,
    .put_i32 = ufr_enc_ros_humble_put_i32,
    .put_f32 = ufr_enc_ros_humble_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_ecr_ros_humble_put_cmd,
    .put_str = ufr_enc_ros_humble_put_str,
    .put_raw = NULL,

    .enter = ufr_ros_topic_enter_array,
    .leave = ufr_ros_topic_leave_array,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros_humble_new_laser_scan(link_t* link, int type) {
    link->enc_api = &ufr_enc_ros_api;
    return UFR_OK;
}