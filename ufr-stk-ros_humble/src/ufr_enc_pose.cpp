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
// #include "geometry_msgs/msg/pose_stamped.hpp"
#include "ufr_gtw_ros_humble.hpp"
#include "turtlesim/msg/pose.hpp"

struct ll_enc_obj {
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher;
    turtlesim::msg::Pose message;
    int index;

    ll_enc_obj() : index{0} {}
};

// ============================================================================
//  Pose Message Driver
// ============================================================================

static
int ufr_enc_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    std::string topic_name = ufr_args_gets(args, "@topic", "topico");
    ll_enc_obj* enc_obj = new ll_enc_obj();

    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    enc_obj->publisher = gtw_obj->m_node->create_publisher<turtlesim::msg::Pose>(topic_name, 10);
    link->enc_obj = enc_obj;
    ufr_info(link, "loaded encoder for turtlesim/Pose");

    return UFR_OK;
}

static
void ufr_enc_ros_humble_close(link_t* link) {
    
}

static
int ufr_enc_ros_put_u32(link_t* link, uint32_t val) {
    ll_enc_obj* enc_obj = (ll_enc_obj*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.x = val; break;
            case 1: enc_obj->message.y = val; break;
            case 2: enc_obj->message.theta = val; break;
            case 3: enc_obj->message.linear_velocity = val; break;
            case 4: enc_obj->message.angular_velocity = val; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_ros_put_i32(link_t* link, int32_t val) {
    ll_enc_obj* enc_obj = (ll_enc_obj*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.x = val; break;
            case 1: enc_obj->message.y = val; break;
            case 2: enc_obj->message.theta = val; break;
            case 3: enc_obj->message.linear_velocity = val; break;
            case 4: enc_obj->message.angular_velocity = val; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_ros_put_f32(link_t* link, float val) {
    ll_enc_obj* enc_obj = (ll_enc_obj*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.x = val; break;
            case 1: enc_obj->message.y = val; break;
            case 2: enc_obj->message.theta = val; break;
            case 3: enc_obj->message.linear_velocity = val; break;
            case 4: enc_obj->message.angular_velocity = val; break;
            default: break;
        }
        enc_obj->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_ros_put_str(link_t* link, const char* val_str) {
    double val = atof(val_str);

    ll_enc_obj* enc_obj = (ll_enc_obj*) link->enc_obj;
    switch ( enc_obj->index ) {
        case 0: enc_obj->message.x = val; break;
        case 1: enc_obj->message.y = val; break;
        case 2: enc_obj->message.theta = val; break;
        case 3: enc_obj->message.linear_velocity = val; break;
        case 4: enc_obj->message.angular_velocity = val; break;
    }
    enc_obj->index += 1;

    return UFR_OK;
}

static
int ufr_enc_ros_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
    ll_enc_obj* enc_obj = (ll_enc_obj*) link->enc_obj;
    if ( type == 'i' ) {
        
    } else if ( type == 'f' ) {
        
    }
    return 0;
}

static
int ufr_enc_ros_put_cmd(link_t* link, char cmd) {
    ll_enc_obj* enc_obj = (ll_enc_obj*) link->enc_obj;
    if ( cmd == '\n' ) {
        enc_obj->publisher->publish(enc_obj->message);
        enc_obj->index = 0;
        ufr_info(link, "sent message turtlesim/Pose");
    }
    return 0;
}

static
ufr_enc_api_t ufr_enc_ros_driver = {
    .boot = ufr_enc_ros_humble_boot,
    .close = ufr_enc_ros_humble_close,
    .clear = NULL,
    .set_header = NULL,

    .put_u8 = NULL,
    .put_i8 = NULL,
    .put_cmd = ufr_enc_ros_put_cmd,
    .put_str = ufr_enc_ros_put_str,
    .put_raw = NULL,

    .put_u32 = ufr_enc_ros_put_u32,
    .put_i32 = ufr_enc_ros_put_i32,
    .put_f32 = ufr_enc_ros_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_arr = NULL,
    .put_mat = NULL,

    .enter_array = NULL,
    .leave_array = NULL,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros_humble_new_pose(link_t* link, int type) {
    link->enc_api = &ufr_enc_ros_driver;
    return UFR_OK;
}

