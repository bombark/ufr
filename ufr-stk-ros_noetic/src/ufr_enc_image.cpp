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
#include <ros/ros.h>
// #include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include "ufr_gtw_ros_noetic.hpp"

struct Encoder {
    ros::Publisher publisher;
    sensor_msgs::Image message;
    uint8_t index;
    uint32_t index2;

    Encoder() : index{0}, index2{0} {
    }

    void clear() {
        index = 0;
    }
};

// ============================================================================
//  String Message Driver
// ============================================================================

static
int ufr_enc_ros_boot(link_t* link, const ufr_args_t* args) {
    std::string topic = ufr_args_gets(args, "@topic", "image");
    Gateway* gtw_obj = (Gateway*) link->gtw_obj;
    Encoder* enc_obj = new Encoder();
    enc_obj->publisher = gtw_obj->node.advertise<sensor_msgs::Image>(topic, 10);
    link->enc_obj = enc_obj;
    return UFR_OK;
}

static
void ufr_enc_ros_close(link_t* link) {
    
}

static
int ufr_enc_ros_put_u32(link_t* link, uint32_t val) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.height = val; enc_obj->index += 1; break;
            case 1: enc_obj->message.width = val; enc_obj->index += 1; break;
            case 2: enc_obj->message.data[enc_obj->index2++] = val; break;
        }
    }
    return 0;
}

static
int ufr_enc_ros_put_i32(link_t* link, int32_t val) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.height = val; enc_obj->index += 1; break;
            case 1: enc_obj->message.width = val; enc_obj->index += 1; break;
            case 2: enc_obj->message.step = val; enc_obj->index += 1; break;
            case 3: enc_obj->message.data[enc_obj->index2++] = val; break;
        }
    }
    return 0;
}

static
int ufr_enc_ros_put_f32(link_t* link, float val) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {

    }
    return 0;
}

static
int ufr_enc_ros_put_str(link_t* link, const char* val) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {
        enc_obj->message.encoding = val;
    }
    return 0;
}

static
int ufr_enc_ros_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( type == 'i' ) {

    } else if ( type == 'f' ) {

    }
    return 0;
}

static
int ufr_enc_ros_put_cmd(link_t* link, char cmd) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( cmd == '\n' ) {
        enc_obj->publisher.publish(enc_obj->message);
        enc_obj->clear();
        ufr_log(link, "sent image message");
    }
    return 0;
}

static
int ufr_ros_enter_array(struct _link* link, size_t maxsize) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj->index == 3 ) {
        enc_obj->message.data.resize(maxsize);
        enc_obj->index2 = 0;
    } else if ( enc_obj->index == 8 ) {
        // enc_obj->message.intensities.resize(maxsize);
        // enc_obj->index2 = 0;
    } else {
        printf("errooo\n");
        return 1;
    }
    return UFR_OK;
}

static
int ufr_ros_leave_array(struct _link* link) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    enc_obj->index += 1;
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_ros = {
    .boot = ufr_enc_ros_boot,
    .close = ufr_enc_ros_close,
    .clear = NULL,
    .set_header = NULL,

    .put_u8 = NULL,
    .put_i8 = NULL,
    .put_cmd = ufr_enc_ros_put_cmd,
    .put_str = ufr_enc_ros_put_str,
    .put_raw = NULL,

    .put_u32 = ufr_enc_ros_put_u32,
    .put_i32 = ufr_enc_ros_put_i32,
    .put_f32 = ufr_enc_ros_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_arr = ufr_enc_ros_put_arr,
    .put_mat = NULL,

    .enter_array = ufr_ros_enter_array,
    .leave_array = ufr_ros_leave_array,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros_melodic_new_image(link_t* link, const int type) {
    link->enc_api = &ufr_enc_ros;
    return 0;
}
