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
#include <ros/ros.h>

#include "ufr_gtw_ros_melodic.hpp"

int g_ros_count = 0;

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
        int argc = 1;
        char* argv[] = {"node"};
        ros::init(argc, argv, "node");
        g_ros_count = 1;
    }
    
    Gateway* gtw = new Gateway();
    link->gtw_obj = gtw;

    return UFR_OK;
}

int ufr_ros_topic_start(link_t* link, int type, const ufr_args_t* args) {
    std::string msg = ufr_args_gets(args, "@msg", "");

    if ( type == UFR_START_SUBSCRIBER ) {

    } else if ( type == UFR_START_PUBLISHER ) {
        if ( msg == "string" ) {
            // ufr_enc_ros_noetic_new_string(link, UFR_START_PUBLISHER);
            // ufr_boot_enc(link, args);
            sys_ufr_load(link, "enc", "ros_melodic:string", UFR_START_PUBLISHER, args);
            ufr_log(link, "loaded ros_melodic:string");
        } else if ( msg == "image" ) {
            sys_ufr_load(link, "enc", "ros_melodic:image", UFR_START_PUBLISHER, args);
            ufr_log(link, "loaded ros_melodic:image");
        }
    }
    return UFR_OK;
}

void ufr_ros_topic_stop(link_t* link, int type) {
    
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

}

static
int ufr_ros_topic_recv_async(link_t* link) {

}

static
ufr_gtw_api_t ufr_ros_melodic_topic_drv = {
    .name = "ROS/Melodic:Topic",
    .type = ufr_ros_topic_type,
    .state = ufr_ros_topic_state,
    .size = ufr_ros_topic_size,
    .boot = ufr_ros_topic_boot,
    .start = ufr_ros_topic_start,
    .stop = ufr_ros_topic_stop,
    .copy = NULL,
    .read = ufr_ros_topic_read,
    .write = ufr_ros_topic_write,
    .recv = ufr_ros_topic_recv,
    .recv_async = ufr_ros_topic_recv_async,
};

// ======================================================================================
//  Constructors
// ======================================================================================

extern "C"
int ufr_gtw_ros_melodic_new_topic(link_t* out, int type) {
    out->gtw_api = &ufr_ros_melodic_topic_drv;
    return UFR_OK;
}
