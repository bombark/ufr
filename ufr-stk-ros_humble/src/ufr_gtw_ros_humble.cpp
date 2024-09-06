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

int ufr_ros_topic_type(const link_t* link) {
    return 0;
}

int ufr_ros_topic_state(const link_t* link) {
    return 0;
}

size_t ufr_ros_topic_size(const link_t* link, int type) {
    return 0;
}

int ufr_ros_topic_boot(link_t* link, const ufr_args_t* args) {
    if ( g_ros_count == 0 ) {
        ufr_info(link, "ROS2 start");
        const char *argv[] = {"./teste1", NULL};
        rclcpp::init(1, argv);
    }
    g_ros_count += 1;

    ll_gateway_t* gtw_obj = new ll_gateway_t();
    link->gtw_obj = (void*) gtw_obj;
    return UFR_OK;
}

int ufr_ros_topic_start(link_t* link, int type, const ufr_args_t* args) {
    std::string msg = ufr_args_gets(args, "@msg", "");

    if ( type == UFR_START_SUBSCRIBER ) {
        ufr_args_t args;
        args.text = "";
        // ufr_load_decoder(link, "ros2:twist", &args);

    } else if ( type == UFR_START_PUBLISHER ) {

        if ( msg == "twist" ) {
            sys_ufr_load(link, "enc", "ros_humble:twist", UFR_START_PUBLISHER, args);
            ufr_log(link, "loaded ros_humble:twist");
        } else if ( msg == "pose" ) {
            sys_ufr_load(link, "enc", "ros_humble:pose", UFR_START_PUBLISHER, args);
            ufr_log(link, "loaded ros_humble:pode");
        } else if ( msg == "string" ) {
            sys_ufr_load(link, "enc", "ros_humble:string", UFR_START_PUBLISHER, args);
            ufr_log(link, "loaded ros_humble:string");
        } else if ( msg == "image" ) {
            sys_ufr_load(link, "enc", "ros_humble:image", UFR_START_PUBLISHER, args);
            ufr_log(link, "loaded ros_humble:image");
        } else if ( msg == "laser_scan" ) {
            sys_ufr_load(link, "enc", "ros_humble:laser_scan", UFR_START_PUBLISHER, args);
            ufr_log(link, "loaded ros_humble:laser_scan");
        } else {
            ufr_log(link, "error");
        }
        
    } 
    return UFR_OK;
}

void ufr_ros_topic_stop(link_t* link, int type) {
    if ( g_ros_count <= 1 ) {
        rclcpp::shutdown();
        g_ros_count = 0;
    } else {
        g_ros_count -= 1;
    }
}

static
size_t ufr_ros_topic_read(link_t* link, char* buffer, size_t length) {
	return 0;
}

static
size_t ufr_ros_topic_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int ufr_ros_topic_recv(link_t* link) {
    link->dcr_api->recv_cb(link, NULL, 0U);
    return true;
}

ufr_gtw_api_t ufr_ros_humble_topic_drv = {
    .name = "ROS:Topic",
    .type = ufr_ros_topic_type,
    .state = ufr_ros_topic_state,
    .size = ufr_ros_topic_size,
    .boot = ufr_ros_topic_boot,
    .start = ufr_ros_topic_start,
    .stop = ufr_ros_topic_stop,
    .read = ufr_ros_topic_read,
    .write = ufr_ros_topic_write,
    .recv = ufr_ros_topic_recv
};

// ======================================================================================
//  Constructors
// ======================================================================================

extern "C"
int ufr_gtw_ros_humble_new_topic(link_t* out, int type) {
    out->gtw_api = &ufr_ros_humble_topic_drv;
    return UFR_OK;
}