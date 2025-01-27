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
#include "sensor_msgs/msg/image.hpp"
#include "ufr_gtw_ros_humble.hpp"

typedef ufr_ros_decoder_t<sensor_msgs::msg::Image> ll_decoder_t;

// ============================================================================l
//  Image - Private
// ============================================================================

static
int ufr_dcr_ros_humble_get_u32(link_t* link, uint32_t* val, int nitems) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
	if ( dcr ) {

        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_i32(link_t* link, int32_t* val, int nitems) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
	if ( dcr ) {
        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_f32(link_t* link, float* val, int nitems) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
	if ( dcr ) {

        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_str(link_t* link, std::string& val) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
	if ( dcr ) {

	}
	return 0;
}

static 
int ufr_dcr_ros_humble_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    ll_gateway_t* gtw = (ll_gateway_t*) link->gtw_obj;
    return dcr->recv(gtw);
}

static 
int ufr_dcr_ros_humble_recv_async_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    ll_gateway_t* gtw = (ll_gateway_t*) link->gtw_obj;
    return dcr->recv_async(gtw);
}

static
ufr_dcr_api_t ufr_dcr_ros_driver = {
    .recv_cb = ufr_dcr_ros_humble_recv_cb,
    .get_u32 = ufr_dcr_ros_humble_get_u32,
    .get_i32 = ufr_dcr_ros_humble_get_i32,
    .get_f32 = ufr_dcr_ros_humble_get_f32,
    // .get_str = ufr_dcr_ros_humble_get_str
};

// ============================================================================
//  Image - Public
// ============================================================================

extern "C"
int ufr_dcr_ros_humble_new_image(link_t* link, int type) {
	link->dcr_api = &ufr_dcr_ros_driver;
    return UFR_OK;
}

