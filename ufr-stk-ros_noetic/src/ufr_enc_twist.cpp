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
#include <geometry_msgs/Twist.h>

#include "ufr_gtw_ros_noetic.hpp"

struct Encoder {
    ros::Publisher publisher;
    geometry_msgs::Twist message;
    uint8_t index;

    Encoder() : index{0} {
    }

    void clear() {
        index = 0;
        message.linear.x = 0;
        message.linear.y = 0;
        message.linear.z = 0;
        message.angular.x = 0;
        message.angular.y = 0;
        message.angular.z = 0;
    }
};

// ============================================================================
//  String Message Driver
// ============================================================================

static
int ufr_enc_ros_boot(link_t* link, const ufr_args_t* args) {
    Gateway* gtw_obj = (Gateway*) link->gtw_obj;
	Encoder* enc_obj = new Encoder();
    enc_obj->publisher = gtw_obj->node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
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
            case 0: enc_obj->message.linear.x = val; break;
            case 1: enc_obj->message.linear.y = val; break;
            case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.angular.x = val; break;
            case 4: enc_obj->message.angular.y = val; break;
            case 5: enc_obj->message.angular.z = val; break;
        }
        enc_obj->index += 1;
	}
	return 0;
}

static
int ufr_enc_ros_put_i32(link_t* link, int32_t val) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.linear.x = val; break;
            case 1: enc_obj->message.linear.y = val; break;
            case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.angular.x = val; break;
            case 4: enc_obj->message.angular.y = val; break;
            case 5: enc_obj->message.angular.z = val; break;
        }
        enc_obj->index += 1;
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
        // enc_obj->message.data.clear();
        enc_obj->clear();
        ufr_log(link, "sent twist message");
	}
	return 0;
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

    .enter_array = NULL,
    .leave_array = NULL,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros_noetic_new_twist(link_t* link, const int type) {
	link->enc_api = &ufr_enc_ros;
	return 0;
}