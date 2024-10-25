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
#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include "ufr_gtw_ros_melodic.hpp"

struct Decoder {
    ros::Subscriber m_sub;
    std_msgs::Int16 m_msg;
    uint8_t count_read;
    uint8_t count_recv;

    Decoder(Gateway* gtw, const std::string topic_name, int buffer_size) {
        count_read = 0;
        count_recv = 0;
        m_sub = gtw->node.subscribe (topic_name, buffer_size,
            &Decoder::callback, this);
    }

    void callback(const std_msgs::Int16::ConstPtr& msg) {
        m_msg.data = msg->data;
        count_recv += 1;
        // ROS_INFO("Recebido: %s", msg->data.c_str());
    }
};

// ============================================================================
//  I16 - Private
// ============================================================================

static
int ufr_dcr_ros_boot(link_t* link, const ufr_args_t* args) {
    const std::string topic_name = ufr_args_gets(args, "@topic", "topic");

    Gateway* gtw = (Gateway*) link->gtw_obj;
    Decoder* dcr = new Decoder(gtw, topic_name, 50);
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_ros_close(link_t* link) {
    
}

static
char ufr_dcr_ros_get_type(link_t* link) {
    return 'i';
}

static
size_t ufr_dcr_ros_get_size(link_t* link) {
    return 1;
}

static
int ufr_dcr_ros_get_u32(link_t* link, uint32_t* val) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        *val = (uint32_t) dcr->m_msg.data;
    }
    return UFR_OK;
}

static
int ufr_dcr_ros_get_i32(link_t* link, int32_t* val) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        *val = (int32_t) dcr->m_msg.data;
    }
    return UFR_OK;
}

static
int ufr_dcr_ros_get_f32(link_t* link, float* val) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        *val = (float) dcr->m_msg.data;
    }
    return UFR_OK;
}

static
int ufr_dcr_ros_get_str(link_t* link, char* val, size_t size) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        snprintf(val, size, "%d", dcr->m_msg.data);
    }
    return UFR_OK;
}

static 
void ufr_dcr_ros_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    Gateway* gtw = (Gateway*) link->gtw_obj;
/*    uint8_t count = dcr->count_read;
    do {
        ros::spinOnce();
    } while ( dcr->m_is_received == false );*/
}

static 
int ufr_dcr_ros_recv_async_cb(link_t* link, char* msg_data, size_t msg_size) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    Gateway* gtw = (Gateway*) link->gtw_obj;

    if ( dcr->count_read == dcr->count_recv ) {
        ros::spinOnce();
        return -1;
    }

    const uint8_t count_read_1 = dcr->count_read+1;
    if ( count_read_1 == dcr->count_recv ) {
        dcr->count_read = count_read_1;
        return UFR_OK;
    } else {
        printf("ERROR ufr_dcr_ros_recv_async_cb\n");
    }

    return -1;
}

static
ufr_dcr_api_t ufr_dcr_ros_driver = {
    .boot = ufr_dcr_ros_boot,
    .close = ufr_dcr_ros_close,
    .recv_cb = ufr_dcr_ros_recv_cb,
    .recv_async_cb = ufr_dcr_ros_recv_async_cb,
    .next = NULL,
    .get_type = ufr_dcr_ros_get_type,
    .get_size = ufr_dcr_ros_get_size,
    .get_raw_ptr = NULL,
    .get_raw = NULL,
    .get_str = ufr_dcr_ros_get_str,
    .get_u32 = ufr_dcr_ros_get_u32,
    .get_i32 = ufr_dcr_ros_get_i32,
    .get_f32 = ufr_dcr_ros_get_f32,
    .enter = NULL,
    .leave = NULL
};

// ============================================================================
//  Twist - Public
// ============================================================================

extern "C"
int ufr_dcr_ros_melodic_new_i16(link_t* link, int type) {
    link->dcr_api = &ufr_dcr_ros_driver;
    return UFR_OK;
}

