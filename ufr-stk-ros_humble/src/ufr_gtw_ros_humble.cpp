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

// ======================================================================================
//  Header
// ======================================================================================

#include <memory>
#include <ufr.h>

#include "ufr_gtw_ros_humble.hpp"

size_t g_ros_count = 0;

// ======================================================================================
//  Subscribe
// ======================================================================================

int lt_ros_topic_type(const link_t* link) {
    return 0;
}

int lt_ros_topic_state(const link_t* link) {
    return 0;
}

size_t lt_ros_topic_size(const link_t* link, int type) {
    return 0;
}

int lt_ros_topic_boot(link_t* link, const lt_args_t* args) {
    ll_gateway_t* gw_obj = new ll_gateway_t();
    link->gw_obj = (void*) gw_obj;
    return LT_OK;
}

int lt_ros_topic_start(link_t* link, int type, const lt_args_t* args) {
    if ( type == LT_START_SUBSCRIBER ) {
        std::string msg_format = lt_args_gets(args, "@msg", "");

        lt_args_t args;
        args.text = "";
        // lt_load_decoder(link, "ros2:twist", &args);

    } else if ( type == LT_START_PUBLISHER ) {
        
    } 
    return LT_OK;
}

void lt_ros_topic_stop(link_t* link, int type) {
    if ( g_ros_count <= 1 ) {
        rclcpp::shutdown();
        g_ros_count = 0;
    } else {
        g_ros_count -= 1;
    }
}

static
size_t lt_ros_topic_read(link_t* link, char* buffer, size_t length) {
	return 0;
}

static
size_t lt_ros_topic_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
bool lt_ros_topic_recv(link_t* link) {
    link->dec_api->recv(link, NULL, 0U);
    return true;
}

lt_api_t lt_ros_topic_api = {
    .name = "ROS:Topic",
    .type = lt_ros_topic_type,
    .state = lt_ros_topic_state,
    .size = lt_ros_topic_size,
    .start = lt_ros_topic_start,
    .stop = lt_ros_topic_stop,
    .read = lt_ros_topic_read,
    .write = lt_ros_topic_write,
    .recv = lt_ros_topic_recv
};

// ======================================================================================
//  Constructors
// ======================================================================================

extern "C"
int lt_new_ros2_topic(link_t* out, const lt_args_t* args) {
    if ( g_ros_count == 0 ) {
        lt_info(out, "ROS2 start");
        const char *argv[] = {"./teste1", NULL};
        rclcpp::init(1, argv);
    }
    g_ros_count += 1;
    out->gw_api = &lt_ros_topic_api;
    return lt_ros_topic_boot(out, args);
}