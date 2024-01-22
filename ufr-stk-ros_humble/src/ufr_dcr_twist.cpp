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
#include "geometry_msgs/msg/twist.hpp"
#include "ufr_gtw_ros_humble.hpp"

#define MESSAGE_SIZE 6

using std::placeholders::_1;

const size_t g_translation[MESSAGE_SIZE] = {
    offsetof(geometry_msgs::msg::Twist, linear.x),
    offsetof(geometry_msgs::msg::Twist, linear.y),
    offsetof(geometry_msgs::msg::Twist, linear.z),
    offsetof(geometry_msgs::msg::Twist, angular.x),
    offsetof(geometry_msgs::msg::Twist, angular.y),
    offsetof(geometry_msgs::msg::Twist, angular.z)
};

struct ll_decoder_t {
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    geometry_msgs::msg::Twist message;
    size_t index = 0;
    volatile bool is_received = false;

    size_t msg_format_size = 6;

    void topic_callback(const geometry_msgs::msg::Twist& msg) {
        printf("OPa %g\n", msg.linear.x);
        this->message = msg;
        is_received = true;
    }
};

// ============================================================================
//  Twist - Private
// ============================================================================

bool lt_dec_ros_twist_set_format(link_t* link, const char* msg_format) {
    return true;
}

static
int lt_dec_ros_twist_get_u32(link_t* link, uint32_t* val) {
	ll_decoder_t* dec_obj = (ll_decoder_t*) link->dec_obj;
	if ( dec_obj ) {
		// get the pointer according to the index position
        uint8_t* msg_u8_ptr = (uint8_t*) &dec_obj->message;        
        msg_u8_ptr += g_translation[dec_obj->index];
        double* msg_f64_ptr = (double*) msg_u8_ptr;
        *val = *msg_f64_ptr;

        // update the index
        dec_obj->index += 1;
	}
	return 0;
}

static
int lt_dec_ros_twist_get_i32(link_t* link, int32_t* val) {
	ll_decoder_t* dec_obj = (ll_decoder_t*) link->dec_obj;
	if ( dec_obj ) {
        if ( dec_obj->index >= MESSAGE_SIZE )
            return 0;

        // get the pointer according to the index position
        uint8_t* msg_u8_ptr = (uint8_t*) &dec_obj->message;        
        msg_u8_ptr += g_translation[dec_obj->index];
        double* msg_f64_ptr = (double*) msg_u8_ptr;
        *val = *msg_f64_ptr;

        // update the index
        dec_obj->index += 1;
	}
	return 0;
}

static
int lt_dec_ros_twist_get_f32(link_t* link, float* val) {
	ll_decoder_t* dec_obj = (ll_decoder_t*) link->dec_obj;
	if ( dec_obj ) {
		// get the pointer according to the index position
        uint8_t* msg_u8_ptr = (uint8_t*) &dec_obj->message;        
        msg_u8_ptr += g_translation[dec_obj->index];
        double* msg_f64_ptr = (double*) msg_u8_ptr;
        *val = *msg_f64_ptr;

        // update the index
        dec_obj->index += 1;
	}
	return 0;
}

static
int lt_dec_ros_twist_get_str(link_t* link, const char* val) {
	ll_decoder_t* dec_obj = (ll_decoder_t*) link->dec_obj;
	if ( dec_obj ) {

	}
	return 0;
}

static void lt_dec_ros_twist_recv(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dec_obj = (ll_decoder_t*) link->dec_obj;
    dec_obj->index = 0;

    ll_gateway_t* gw_obj = (ll_gateway_t*) link->gw_obj;
    while ( dec_obj->is_received == false ) {
        rclcpp::spin_some(gw_obj->node);
    }
    dec_obj->is_received = false;
}

static
lt_decoder_api_t lt_dec_ros_twist = {
    .recv = lt_dec_ros_twist_recv,
	.get_u32 = lt_dec_ros_twist_get_u32,
	.get_i32 = lt_dec_ros_twist_get_i32,
	.get_f32 = lt_dec_ros_twist_get_f32,
	// .get_str = lt_enc_ros_twist_get_str
};

// ============================================================================
//  Twist - Public
// ============================================================================

extern "C"
int lt_new_ros2_twist(link_t* link, const lt_args_t* args) {

	link->dec_api = &lt_dec_ros_twist;

    std::string topic_name = lt_args_gets(args, "@topic", "topico");

    ll_gateway_t* gw_obj = (ll_gateway_t*) link->gw_obj;
	ll_decoder_t* dec_obj = new ll_decoder_t();
    dec_obj->subscription = gw_obj->node->create_subscription<geometry_msgs::msg::Twist>(
        topic_name, 10, std::bind(&ll_decoder_t::topic_callback, dec_obj, _1));
	link->dec_obj = dec_obj;

	return 0;
}

