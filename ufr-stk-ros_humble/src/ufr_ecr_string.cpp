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
#include "std_msgs/msg/string.hpp"
#include "ufr_gtw_ros_humble.hpp"

struct ll_encoder_t {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    std_msgs::msg::String message;
};

// ============================================================================
//  String Message Driver
// ============================================================================

static
int lt_enc_ros_string_put_u32(link_t* link, uint32_t val) {
	ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
	if ( enc_obj ) {
		
	}
	return 0;
}

static
int lt_enc_ros_string_put_i32(link_t* link, int32_t val) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {

    }
    return 0;
}

static
int lt_enc_ros_string_put_f32(link_t* link, float val) {
	ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
	if ( enc_obj ) {
		
	}
	return 0;
}

static
int lt_enc_ros_string_put_str(link_t* link, const char* val) {
	ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
	if ( enc_obj ) {
		enc_obj->message.data += val;
	}
	return 0;
}

static
int lt_enc_ros_string_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
	ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
	if ( type == 'i' ) {
		
	} else if ( type == 'f' ) {
		
	}
    return 0;
}

static
int lt_enc_ros_string_put_cmd(link_t* link, char cmd) {
	ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
	if ( cmd == '\n' ) {
		enc_obj->publisher->publish(enc_obj->message);
        enc_obj->message.data = "";
	}
	return 0;
}

static
lt_encoder_api_t lt_enc_ros_string = {
	.put_u32 = lt_enc_ros_string_put_u32,
	.put_i32 = lt_enc_ros_string_put_i32,
	.put_f32 = lt_enc_ros_string_put_f32,
	.put_str = lt_enc_ros_string_put_str,
    .put_cmd = lt_enc_ros_string_put_cmd,
	.put_arr = lt_enc_ros_string_put_arr
};

// ============================================================================
//  Public
// ============================================================================

extern "C" 
int ufr_new_ecr_ros_humble_string(link_t* link, const lt_args_t* args) {
	link->enc_api = &lt_enc_ros_string;

    ll_gateway_t* gw_obj = (ll_gateway_t*) link->gw_obj;
	ll_encoder_t* enc_obj = new ll_encoder_t();
    enc_obj->publisher = gw_obj->m_node->create_publisher<std_msgs::msg::String>("topic", 10);

	link->enc_obj = enc_obj;

	return 0;
}