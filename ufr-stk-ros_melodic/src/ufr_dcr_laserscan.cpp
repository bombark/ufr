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
#include <sensor_msgs/LaserScan.h>
#include "ufr_gtw_ros_melodic.hpp"


#define MAX 8

struct Decoder {
    ros::Subscriber m_sub;
    sensor_msgs::LaserScan* m_message;
    sensor_msgs::LaserScan m_stack[MAX];

    uint8_t m_head;
    uint8_t m_tail;
    uint8_t m_size;

    uint8_t index;
    uint32_t index2;

    union {
        uint64_t* val_u64;
        int64_t* val_i64;
        float* val_f32;
    };


    Decoder(Gateway* gtw, const std::string topic_name, int buffer_size) {
        m_head = 0;
        m_tail = 0;
        m_size = 0;
        m_message = &m_stack[0];

        m_sub = gtw->node.subscribe (topic_name, buffer_size,
            &Decoder::callback, this);
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if ( m_size >= MAX ) {
            ROS_INFO("Stack full");
            return;
        }

        m_stack[m_head] = *msg;
        m_head = (m_head + 1) % MAX;
        m_size += 1;
        // ROS_INFO("Recebido: %s", msg->data.c_str());
    }
};


// ============================================================================
//  Image - Private
// ============================================================================

static
int ufr_dcr_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    const std::string topic_name = ufr_args_gets(args, "@topic", "topic");

    Gateway* gtw = (Gateway*) link->gtw_obj;
    Decoder* dcr = new Decoder(gtw, topic_name, 50);
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_ros_humble_close(link_t* link) {
}


static 
int ufr_dcr_ros_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    Gateway* gtw = (Gateway*) link->gtw_obj;
    while ( dcr->m_size == 0 ) {
        ros::spinOnce();
    }

    dcr->m_message = &dcr->m_stack[ dcr->m_tail ];
    dcr->m_tail = (dcr->m_tail + 1) % MAX;
    dcr->m_size -= 1;
    return UFR_OK;
}

static 
int ufr_dcr_ros_recv_async_cb(link_t* link, char* msg_data, size_t msg_size) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    Gateway* gtw = (Gateway*) link->gtw_obj;

    if ( dcr->m_size == 0 ) {
        ros::spinOnce();
        return -1;
    }

    dcr->m_message = &dcr->m_stack[ dcr->m_tail ];
    dcr->m_tail = (dcr->m_tail + 1) % MAX;
    dcr->m_size -= 1;
    return UFR_OK;
}

static 
int ufr_dcr_ros_humble_next(link_t* link) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    dcr->index += 1;
}

static
char ufr_dcr_ros_humble_get_type(link_t* link) {
    static const char* format = "fffffffaa";
    const Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr && dcr->index < 9 ) {
        return format[ dcr->index ];
    }
    return 0;
}

static
size_t ufr_dcr_ros_humble_get_nitems(link_t* link) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        switch (dcr->index) {
            case 0: return 1;
            case 1: return 1;
            case 2: return 1;
            case 3: return 1;
            case 4: return 1;
            case 5: return 1;
            case 6: return 1;
            case 7: return dcr->m_message->ranges.size();
            case 8: return dcr->m_message->intensities.size();
            default: return 0;
        }
    }
    return 0;
}

static
int ufr_dcr_ros_humble_get_u32(link_t* link, uint32_t* val, int nitems) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        switch (dcr->index) {
            case 0: *val = dcr->m_message->angle_min; dcr->index += 1; break;
            case 1: *val = dcr->m_message->angle_max; dcr->index += 1; break;
            case 2: *val = dcr->m_message->angle_increment; dcr->index += 1; break;
            case 3: *val = dcr->m_message->time_increment; dcr->index += 1; break;
            case 4: *val = dcr->m_message->scan_time; dcr->index += 1; break;
            case 5: *val = dcr->m_message->range_min; dcr->index += 1; break;
            case 6: *val = dcr->m_message->range_max; dcr->index += 1; break;
            case 7: *val = dcr->m_message->ranges[dcr->index2++]; break;
            case 8: *val = dcr->m_message->intensities[dcr->index2++]; break;
            default: break;
        }
        // update the index
        dcr->index += 1;
    }
    return 0;
}

static
int ufr_dcr_ros_humble_get_i32(link_t* link, int32_t* val, int nitems) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        
        

        // update the index
        dcr->index += 1;
    }
    return 0;
}

static
int ufr_dcr_ros_humble_get_f32(link_t* link, float* val, int nitems) {
    *val = 0.0;
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {
        switch (dcr->index) {
            case 0: *val = dcr->m_message->angle_min; dcr->index += 1; break;
            case 1: *val = dcr->m_message->angle_max; dcr->index += 1; break;
            case 2: *val = dcr->m_message->angle_increment; dcr->index += 1; break;
            case 3: *val = dcr->m_message->time_increment; dcr->index += 1; break;
            case 4: *val = dcr->m_message->scan_time; dcr->index += 1; break;
            case 5: *val = dcr->m_message->range_min; dcr->index += 1; break;
            case 6: *val = dcr->m_message->range_max; dcr->index += 1; break;
            case 7: *val = dcr->m_message->ranges[ dcr->index2++ ]; break;
            case 8: *val = dcr->m_message->intensities[ dcr->index2++ ]; break;
            default: break;
        }
        // update the index
        
    }
    return 0;
}

static
int ufr_dcr_ros_humble_get_str(link_t* link, std::string& val) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr ) {

    }
    return 0;
}

static 
int ufr_dcr_ros_humble_enter(link_t* link) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr->index == 7 || dcr->index == 8 ) {
        dcr->index2 = 0;
        return UFR_OK;
    }
    return -1;
}

static 
int ufr_dcr_ros_humble_leave(link_t* link) {
    Decoder* dcr = (Decoder*) link->dcr_obj;
    if ( dcr->index == 7 || dcr->index == 8 ) {
        dcr->index += 1;
        return UFR_OK;
    }
    return -1;
}

static
ufr_dcr_api_t ufr_dcr_ros_driver = {
    .boot = ufr_dcr_ros_humble_boot,
    .close = NULL,
    .recv_cb = ufr_dcr_ros_recv_cb,
    .recv_async_cb = ufr_dcr_ros_recv_async_cb,
    .next = ufr_dcr_ros_humble_next,

    .get_type = NULL,
    .get_nbytes = NULL,
    .get_nitems = NULL,
    .get_raw_ptr = NULL,

    .get_raw = NULL,
    .get_str = NULL,

    .get_u32 = ufr_dcr_ros_humble_get_u32,
    .get_i32 = ufr_dcr_ros_humble_get_i32,
    .get_f32 = ufr_dcr_ros_humble_get_f32,

    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

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

/* 

  ffffaiia
      f  i
      .  .
      f  i


*/




