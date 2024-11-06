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
#include <vector>
#include <ufr.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ufr_gtw_ros_humble.hpp"

struct ll_enc_obj_t {
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    sensor_msgs::msg::Image message;
    int index;

    ll_enc_obj_t() : index{0} {}
};

// ============================================================================
//  Image Message Driver
// ============================================================================

static 
int ufr_ecr_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    std::string topic_name = ufr_args_gets(args, "@topic", "topico");
    ll_enc_obj_t* enc_obj = new ll_enc_obj_t();

    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    enc_obj->publisher = gtw_obj->m_node->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    link->enc_obj = enc_obj;
    ufr_info(link, "loaded encoder for sensor_msgs/Image");

    return UFR_OK;
}

int ufr_ecr_ros_put_raw(struct _link* link, const uint8_t* val, size_t size) {
    ll_enc_obj_t* enc_obj = (ll_enc_obj_t*) link->enc_obj;
    enc_obj->message.data.resize(size);
    memcpy(&enc_obj->message.data[0], val, size);
    return UFR_OK;
}


static
int ufr_enc_ros_humble_put_u32(link_t* link, uint32_t val) {
	ll_enc_obj_t* enc_obj = (ll_enc_obj_t*) link->enc_obj;
	if ( enc_obj ) {
		switch(enc_obj->index) {
            case 0: enc_obj->message.height = val; break;
            case 1: enc_obj->message.width = val; break;
            // case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.is_bigendian = val; break;
            case 4: enc_obj->message.step = val; break;
            // case 5: enc_obj->message.data = val; break;
            default: break;
        }
        enc_obj->index += 1;
	}
	return 0;
}

static
int ufr_enc_ros_humble_put_i32(link_t* link, int32_t val) {
	ll_enc_obj_t* enc_obj = (ll_enc_obj_t*) link->enc_obj;
	if ( enc_obj ) {
		switch(enc_obj->index) {
            case 0: enc_obj->message.height = val; break;
            case 1: enc_obj->message.width = val; break;
            // case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.is_bigendian = val; break;
            case 4: enc_obj->message.step = val; break;
            // case 5: enc_obj->message.angular.z = val; break;
            default: break;
        }
        enc_obj->index += 1;
	}
	return 0;
}

static
int ufr_enc_ros_humble_put_f32(link_t* link, float val) {
	ll_enc_obj_t* enc_obj = (ll_enc_obj_t*) link->enc_obj;
	if ( enc_obj ) {
		switch(enc_obj->index) {
            /*case 0: enc_obj->message.linear.x = val; break;
            case 1: enc_obj->message.linear.y = val; break;
            case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.angular.x = val; break;
            case 4: enc_obj->message.angular.y = val; break;
            case 5: enc_obj->message.angular.z = val; break;*/
            default: break;
        }
        enc_obj->index += 1;
	}
	return 0;
}

static
int ufr_enc_ros_humble_put_str(link_t* link, const char* val) {
	ll_enc_obj_t* enc_obj = (ll_enc_obj_t*) link->enc_obj;
	if ( enc_obj ) {

	}
	return 0;
}

static
int ufr_enc_ros_humble_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
	ll_enc_obj_t* enc_obj = (ll_enc_obj_t*) link->enc_obj;
	if ( type == 'i' ) {
		
	} else if ( type == 'f' ) {
		
	}
    return 0;
}

static
int ufr_ecr_ros_humble_put_cmd(link_t* link, char cmd) {
	ll_enc_obj_t* enc_obj = (ll_enc_obj_t*) link->enc_obj;
	if ( cmd == '\n' ) {
		enc_obj->publisher->publish(enc_obj->message);
        enc_obj->index = 0;
        ufr_info(link, "sent message sensor_msgs/Image");
	}
	return 0;
}

static
ufr_enc_api_t ufr_enc_ros_image = {
    .boot = ufr_ecr_ros_humble_boot,
    .close = NULL,
    .clear = NULL,
    .set_header = NULL,

    .put_u8 = NULL,
    .put_i8 = NULL,
    .put_cmd = ufr_ecr_ros_humble_put_cmd,
    .put_str = ufr_enc_ros_humble_put_str,
    .put_raw = ufr_ecr_ros_put_raw,

    .put_u32 = ufr_enc_ros_humble_put_u32,
    .put_i32 = ufr_enc_ros_humble_put_i32,
    .put_f32 = ufr_enc_ros_humble_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_arr = ufr_enc_ros_humble_put_arr,
    .put_mat = NULL,

    .enter_array = NULL,
    .leave_array = NULL,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_ecr_ros_humble_new_image(link_t* link, const ufr_args_t* args) {
    link->enc_api = &ufr_enc_ros_image;
	return UFR_OK;
}

