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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ufr_gtw_ros_melodic.hpp"


struct Encoder {
    tf::TransformBroadcaster broadcaster;
    tf::Quaternion q;
    std::string frame_id;
    std::string child_frame_id;

    float x,y,th;
    uint8_t index;

    Encoder() : x{0}, y{0}, th{0}, index{0} { }
};

// ============================================================================
//  Pose Message Driver
// ============================================================================

static
int ufr_enc_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    std::string topic_name = ufr_args_gets(args, "@topic", "topico");
    std::string frame_id = ufr_args_gets(args, "@frame", "frame");
    std::string child_frame_id = ufr_args_gets(args, "@child", "child");

    Encoder* enc = new Encoder();
    enc->frame_id = frame_id;
    enc->child_frame_id = child_frame_id;

    Gateway* gtw = (Gateway*) link->gtw_obj;
    // enc_obj->publisher = gtw_obj->m_node->create_publisher<geometry_msgs::msg::Pose>(topic_name, 10);


    link->enc_obj = enc;
    ufr_info(link, "loaded encoder for geometry/twist");

    return UFR_OK;
}

static
void ufr_enc_ros_humble_close(link_t* link) {
    
}

static
int ufr_enc_ros_put_u32(link_t* link, const uint32_t val[], int items) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->x = val[0]; break;
            case 1: enc_obj->y = val[0]; break;
            case 2: enc_obj->th = val[0]; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_ros_put_i32(link_t* link, const int32_t val[], int nitems) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->x = val[0]; break;
            case 1: enc_obj->y = val[0]; break;
            case 2: enc_obj->th = val[0]; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_ros_put_f32(link_t* link, const float val[], int nitems) {
    Encoder* enc_obj = (Encoder*) link->enc_obj;
    switch(enc_obj->index) {
        case 0: enc_obj->x = val[0]; break;
        case 1: enc_obj->y = val[0]; break;
        case 2: enc_obj->th = val[0]; break;
        default: break;
    }
    enc_obj->index += 1;
    return UFR_OK;
}

static
int ufr_enc_ros_put_str(link_t* link, const char* val_str) {
    const double val = atof(val_str);

    Encoder* enc_obj = (Encoder*) link->enc_obj;
    switch ( enc_obj->index ) {
        case 0: enc_obj->x = val; break;
        case 1: enc_obj->y = val; break;
        case 2: enc_obj->th = val; break;
    }
    return UFR_OK;
}

static
int ufr_enc_ros_put_cmd(link_t* link, char cmd) {
    Encoder* enc = (Encoder*) link->enc_obj;
    if ( cmd == '\n' ) {
        // enc_obj->publisher->publish(enc_obj->message);

        //
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(enc->x, enc->y, 0.0));

        // 
        tf::Quaternion q;
        q.setRPY(0, 0, enc->th);
        transform.setRotation(q);

        // send
        enc->broadcaster.sendTransform (
            tf::StampedTransform(transform, ros::Time::now(), enc->frame_id, enc->child_frame_id)
        );
        enc->index = 0;
    }
    return 0;
}

static
ufr_enc_api_t ufr_enc_ros_driver = {
    .boot = ufr_enc_ros_humble_boot,
    .close = ufr_enc_ros_humble_close,
    .clear = NULL,

    .put_u32 = ufr_enc_ros_put_u32,
    .put_i32 = ufr_enc_ros_put_i32,
    .put_f32 = ufr_enc_ros_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_enc_ros_put_cmd,
    .put_str = ufr_enc_ros_put_str,
    .put_raw = NULL,

    .enter = NULL,
    .leave = NULL,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros_melodic_new_pose(link_t* link, int type) {
    link->enc_api = &ufr_enc_ros_driver;
    return UFR_OK;
}

