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
int lt_enc_ros_twist_put_u32(link_t* link, uint32_t val) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( enc_obj ) {
		switch(enc_obj->index) {
            case 0: enc_obj->message.linear.x = val; break;
            case 1: enc_obj->message.linear.y = val; break;
            case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.angular.x = val; break;
            case 4: enc_obj->message.angular.y = val; break;
            case 5: enc_obj->message.angular.z = val; break;
            default: break;
        }
        enc_obj->index += 1;
	}
	return 0;
}

static
int lt_enc_ros_twist_put_i32(link_t* link, int32_t val) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( enc_obj ) {
		switch(enc_obj->index) {
            case 0: enc_obj->message.linear.x = val; break;
            case 1: enc_obj->message.linear.y = val; break;
            case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.angular.x = val; break;
            case 4: enc_obj->message.angular.y = val; break;
            case 5: enc_obj->message.angular.z = val; break;
            default: break;
        }
        enc_obj->index += 1;
	}
	return 0;
}

static
int lt_enc_ros_twist_put_f32(link_t* link, float val) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( enc_obj ) {
		switch(enc_obj->index) {
            case 0: enc_obj->message.linear.x = val; break;
            case 1: enc_obj->message.linear.y = val; break;
            case 2: enc_obj->message.linear.z = val; break;
            case 3: enc_obj->message.angular.x = val; break;
            case 4: enc_obj->message.angular.y = val; break;
            case 5: enc_obj->message.angular.z = val; break;
            default: break;
        }
        enc_obj->index += 1;
	}
	return 0;
}

static
int lt_enc_ros_twist_put_str(link_t* link, const char* val) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( enc_obj ) {

	}
	return 0;
}

static
int lt_enc_ros_twist_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( type == 'i' ) {
		
	} else if ( type == 'f' ) {
		
	}
    return 0;
}

static
int ufr_ecr_ros_humble_put_cmd(link_t* link, char cmd) {
	ll_encoder_twist* enc_obj = (ll_encoder_twist*) link->enc_obj;
	if ( cmd == '\n' ) {
		enc_obj->publisher->publish(enc_obj->message);
        enc_obj->index = 0;
        lt_info(link, "sent message geometry/twist");
	}
	return 0;
}

static
lt_encoder_api_t lt_enc_ros_twist = {
	.put_u32 = lt_enc_ros_twist_put_u32,
	.put_i32 = lt_enc_ros_twist_put_i32,
	.put_f32 = lt_enc_ros_twist_put_f32,
	.put_str = lt_enc_ros_twist_put_str,
    .put_cmd = ufr_ecr_ros_humble_put_cmd,
	.put_arr = lt_enc_ros_twist_put_arr
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_new_ecr_ros_humble_twist(link_t* link, const lt_args_t* args) {
    std::string gw_api_name = lt_api_name(link);

    if ( gw_api_name == "ROS:Topic" ) {
        link->enc_api = &lt_enc_ros_twist;
        std::string topic_name = lt_args_gets(args, "@topic", "topico");
        ll_encoder_twist* enc_obj = new ll_encoder_twist();

        ll_gateway_t* gw_obj = (ll_gateway_t*) link->gw_obj;
        enc_obj->publisher = gw_obj->m_node->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
        link->enc_obj = enc_obj;
        lt_info(link, "loaded encoder for geometry/twist");
    } else {
printf("op\n");
        lt_args_t args;
        args.text = "@sep ;";
        lt_load_encoder(link, "std:csv", &args);
    }

	return LT_OK;
}

