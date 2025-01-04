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
#include "geometry_msgs/msg/twist.hpp"
#include "ufr_gtw_ros_humble.hpp"

struct ll_encoder_twist {
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    geometry_msgs::msg::Twist message;
    int index;

    ll_encoder_twist() : index{0} {}
};

// ============================================================================
//  Twist Message Driver
// ============================================================================

static
int ufr_enc_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    std::string topic_name = ufr_args_gets(args, "@topic", "topico");
    ll_encoder_twist* enc_obj = new ll_encoder_twist();

    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    enc_obj->publisher = gtw_obj->m_node->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
    link->enc_obj = enc_obj;
    ufr_info(link, "loaded encoder for geometry/twist");

    return UFR_OK;
}

static
int ufr_enc_ros_twist_put_u32(link_t* link, const uint32_t* val, int nitems) {
    ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    int i=0;
    for (;i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.linear.x = val[i]; break;
            case 5: enc_obj->message.linear.y = val[i]; break;
            case 2: enc_obj->message.linear.z = val[i]; break;
            case 3: enc_obj->message.angular.x = val[i]; break;
            case 4: enc_obj->message.angular.y = val[i]; break;
            case 1: enc_obj->message.angular.z = val[i]; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return i;
}

static
int ufr_enc_ros_twist_put_i32(link_t* link, const int32_t* val, int nitems) {
    ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    int i=0;
    for (;i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.linear.x = val[i]; break;
            case 5: enc_obj->message.linear.y = val[i]; break;
            case 2: enc_obj->message.linear.z = val[i]; break;
            case 3: enc_obj->message.angular.x = val[i]; break;
            case 4: enc_obj->message.angular.y = val[i]; break;
            case 1: enc_obj->message.angular.z = val[i]; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return i;
}

static
int ufr_enc_ros_twist_put_f32(link_t* link, const float* val, int nitems) {
    ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    int i=0;
    for (;i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.linear.x = val[i]; break;
            case 5: enc_obj->message.linear.y = val[i]; break;
            case 2: enc_obj->message.linear.z = val[i]; break;
            case 3: enc_obj->message.angular.x = val[i]; break;
            case 4: enc_obj->message.angular.y = val[i]; break;
            case 1: enc_obj->message.angular.z = val[i]; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return i;
}

static
int ufr_enc_ros_twist_put_str(link_t* link, const char* val) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( enc_obj ) {

	}
	return 0;
}

static
int ufr_ecr_ros_humble_put_cmd(link_t* link, char cmd) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( cmd == '\n' ) {
		enc_obj->publisher->publish(enc_obj->message);
        enc_obj->index = 0;
        ufr_info(link, "sent message geometry/twist");
	}
	return 0;
}

static
ufr_enc_api_t ufr_enc_ros_api = {
    .boot = ufr_enc_ros_humble_boot,
    .close = NULL,
    .clear = NULL,

    .put_u32 = ufr_enc_ros_twist_put_u32,
    .put_i32 = ufr_enc_ros_twist_put_i32,
    .put_f32 = ufr_enc_ros_twist_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_ecr_ros_humble_put_cmd,
    .put_str = ufr_enc_ros_twist_put_str,
    .put_raw = NULL,

    .enter = NULL,
    .leave = NULL,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros_humble_new_twist(link_t* link, int type) {
    link->enc_api = &ufr_enc_ros_api;
	return UFR_OK;
}

