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

#include <ufr.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ufr_gtw_ros_humble.hpp"

typedef ufr_ros_decoder_t<geometry_msgs::msg::Pose> ll_decoder_t;

/*
const size_t g_translation[6] = {
    offsetof(geometry_msgs::msg::Twist, linear.x),
    offsetof(geometry_msgs::msg::Twist, linear.y),
    offsetof(geometry_msgs::msg::Twist, linear.z),
    offsetof(geometry_msgs::msg::Twist, angular.x),
    offsetof(geometry_msgs::msg::Twist, angular.y),
    offsetof(geometry_msgs::msg::Twist, angular.z)
};
*/

// ============================================================================
//  Twist - Private
// ============================================================================

static
int ufr_dcr_ros_humble_get_u32(link_t* link, uint32_t* val) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dec_obj;
	if ( dcr ) {

        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_i32(link_t* link, int32_t* val) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dec_obj;
	if ( dcr ) {
        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_f32(link_t* link, float* val) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dec_obj;
	if ( dcr ) {

        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_str(link_t* link, std::string& val) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dec_obj;
	if ( dcr ) {

	}
	return 0;
}

static void ufr_dcr_ros_humble_recv(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dec_obj;
    dcr->index = 0;

    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gw_obj;
    while ( dcr->m_is_received == false ) {
        rclcpp::spin_some(gtw_obj->m_node);
    }
    dcr->m_is_received = false;
}

static
lt_decoder_api_t ufr_dcr_ros_driver = {
    .recv = ufr_dcr_ros_humble_recv,
	.get_u32 = ufr_dcr_ros_humble_get_u32,
	.get_i32 = ufr_dcr_ros_humble_get_i32,
	.get_f32 = ufr_dcr_ros_humble_get_f32,
	// .get_str = ufr_dcr_ros_humble_get_str
};

// ============================================================================
//  Twist - Public
// ============================================================================

extern "C"
int ufr_new_dcr_ros_humble_pose(link_t* link, const lt_args_t* args) {
	std::string topic_name = lt_args_gets(args, "@topic", "topico");

    ll_gateway_t* gtw = (ll_gateway_t*) link->gw_obj;
	ll_decoder_t* dcr = new ll_decoder_t(gtw, topic_name);
    link->dec_obj = (void*) dcr;

    return LT_OK;
}

