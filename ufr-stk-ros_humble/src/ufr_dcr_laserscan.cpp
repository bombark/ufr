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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ufr_gtw_ros_humble.hpp"

typedef ufr_ros_decoder_t<sensor_msgs::msg::LaserScan> ll_decoder_t;

// ============================================================================
//  Image - Private
// ============================================================================

static
int ufr_dcr_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;

    std::string topic_name = ufr_args_gets(args, "@topic", "topico");
    ll_decoder_t* dcr_obj = new ll_decoder_t(gtw_obj, topic_name);
    link->dcr_obj = dcr_obj;
    ufr_info(link, "loaded encoder for sensor_msgs/laserscan");
    return UFR_OK;
}

static
void ufr_dcr_ros_humble_close(link_t* link) {
}

static
char ufr_dcr_ros_humble_get_type(link_t* link) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr ) {
        switch (dcr->index) {
            case 0: return 'f';
            case 1: return 'f';
            case 2: return 'f';
            case 3: return 'f';
            case 4: return 'f';
            case 5: return 'f';
            case 6: return 'f';
            case 7: return 'a';
            case 8: return 'a';
            default: return 0;
        }
    }
    return 0;
}

static
size_t ufr_dcr_ros_humble_get_size(link_t* link) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr ) {
        switch (dcr->index) {
            case 0: return 1;
            case 1: return 1;
            case 2: return 1;
            case 3: return 1;
            case 4: return 1;
            case 5: return 1;
            case 6: return 1;
            case 7: return dcr->m_message.ranges.size();
            case 8: return dcr->m_message.intensities.size();
            default: return 0;
        }
    }
    return 0;
}

static
int ufr_dcr_ros_humble_get_u32(link_t* link, uint32_t* val) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr ) {
        switch (dcr->index) {
            case 0: *val = dcr->m_message.angle_min; break;
            case 1: *val = dcr->m_message.angle_max; break;
            case 2: *val = dcr->m_message.angle_increment; break;
            case 3: *val = dcr->m_message.time_increment; break;
            case 4: *val = dcr->m_message.scan_time; break;
            case 5: *val = dcr->m_message.range_min; break;
            case 6: *val = dcr->m_message.range_max; break;
            case 7: *val = dcr->m_message.ranges[0]; break;
            case 8: *val = dcr->m_message.intensities[0]; break;
            default: break;
        }
        // update the index
        dcr->index += 1;
    }
    return 0;
}

static
int ufr_dcr_ros_humble_get_i32(link_t* link, int32_t* val) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
	if ( dcr ) {
        switch (dcr->index) {
            case 0: *val = dcr->m_message.angle_min; break;
            case 1: *val = dcr->m_message.angle_max; break;
            case 2: *val = dcr->m_message.angle_increment; break;
            case 3: *val = dcr->m_message.time_increment; break;
            case 4: *val = dcr->m_message.scan_time; break;
            case 5: *val = dcr->m_message.range_min; break;
            case 6: *val = dcr->m_message.range_max; break;
            case 7: *val = dcr->m_message.ranges[0]; break;
            case 8: *val = dcr->m_message.intensities[0]; break;
            default: break;
        }
        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_f32(link_t* link, float* val) {
    *val = 0.0;
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr ) {
        switch (dcr->index) {
            case 0: *val = dcr->m_message.angle_min; dcr->index += 1; break;
            case 1: *val = dcr->m_message.angle_max; dcr->index += 1; break;
            case 2: *val = dcr->m_message.angle_increment; dcr->index += 1; break;
            case 3: *val = dcr->m_message.time_increment; dcr->index += 1; break;
            case 4: *val = dcr->m_message.scan_time; dcr->index += 1; break;
            case 5: *val = dcr->m_message.range_min; dcr->index += 1; break;
            case 6: *val = dcr->m_message.range_max; dcr->index += 1; break;
            case 7: *val = dcr->m_message.ranges[ dcr->index2++ ]; break;
            case 8: *val = dcr->m_message.intensities[ dcr->index2++ ]; break;
            default: break;
        }
        // update the index
        
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
void ufr_dcr_ros_humble_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    while ( dcr->m_is_received == false ) {
        rclcpp::spin_some(gtw_obj->m_node);
    }
    dcr->m_is_received = false;
}

static 
int ufr_dcr_ros_humble_recv_async_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    dcr->m_is_received = false;
    rclcpp::spin_some(gtw_obj->m_node);
    return ( dcr->m_is_received == true ) ? UFR_OK : -1;
}

static 
int ufr_dcr_ros_humble_enter(link_t* link) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr->index == 7 || dcr->index == 8 ) {
        dcr->index2 = 0;
        return UFR_OK;
    }
    return -1;
}

static 
int ufr_dcr_ros_humble_leave(link_t* link) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    dcr->index += 1;
    return UFR_OK;
}

static
ufr_dcr_api_t ufr_dcr_ros_driver = {
    .boot = ufr_dcr_ros_humble_boot,
    .recv_cb = ufr_dcr_ros_humble_recv_cb,
    .recv_async_cb = ufr_dcr_ros_humble_recv_async_cb,

    .get_type = ufr_dcr_ros_humble_get_type,
    .get_size = ufr_dcr_ros_humble_get_size,
    .get_u32 = ufr_dcr_ros_humble_get_u32,
    .get_i32 = ufr_dcr_ros_humble_get_i32,
    .get_f32 = ufr_dcr_ros_humble_get_f32,
    // .get_str = ufr_dcr_ros_humble_get_str
    .enter = ufr_dcr_ros_humble_enter,
    .leave = ufr_dcr_ros_humble_leave,
};

// ============================================================================
//  Image - Public
// ============================================================================

extern "C"
int ufr_dcr_ros_humble_new_laserscan(link_t* link, int type) {
    link->dcr_api = &ufr_dcr_ros_driver;
    return UFR_OK;
}
