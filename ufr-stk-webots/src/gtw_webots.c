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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <webots/robot.h>

#include "ufr_webots.h"

int g_time_step = 100;

// ============================================================================
//  Loop callback
// ============================================================================

static
int webots_loop_cb(void) {
    wb_robot_step(g_time_step);
    return UFR_OK;
}

// ============================================================================
//  Gateway
// ============================================================================

int    ufr_gtw_webots_type(const link_t* link) {
    return 0;
}

int    ufr_gtw_webots_state(const link_t* link) {
    return 0;
}

size_t ufr_gtw_webots_size(const link_t* link, int type) {
    return 0;
}

int  ufr_gtw_webots_boot(link_t* link, const ufr_args_t* args) {
    // Library initialization
    static uint8_t is_initialized = 0;
    if ( is_initialized == 0 ) {
        wb_robot_init();
        ufr_put_loop_callback( webots_loop_cb );
    }
    is_initialized = 1;

    // Select the encoder ou decoder
    const char* dev_type = ufr_args_gets(args, "@type", "");
    if ( strcmp(dev_type, "motors") == 0 ) {
        ufr_enc_webots_new_motors(link, UFR_START_PUBLISHER);
    } else if ( strcmp(dev_type, "encoders") == 0 ) {
        ufr_dcr_webots_new_encoders(link, UFR_START_SUBSCRIBER);
    } else if ( strcmp(dev_type, "lidar") == 0 ) {
        ufr_dcr_webots_new_lidar(link, UFR_START_SUBSCRIBER);
    } else {
        ufr_log(link, "error");
    }

    return UFR_OK;
}

int  ufr_gtw_webots_start(link_t* link, int type, const ufr_args_t* args) {
    

    return UFR_OK;
}

void ufr_gtw_webots_stop(link_t* link, int type) {

}

int  ufr_gtw_webots_copy(link_t* link, link_t* out) {

}

size_t ufr_gtw_webots_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

size_t ufr_gtw_webots_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

int ufr_gtw_webots_recv(link_t* link) {
    if ( link->dcr_api != NULL ) {
        link->dcr_api->recv_cb(link, NULL, 0);
        return UFR_OK;
    }
    return UFR_OK;
}

int ufr_gtw_webots_recv_async(link_t* link) {

}

int ufr_gtw_webots_send(link_t* link) {

}

int ufr_gtw_webots_accept(link_t* link, link_t* out_client) {

}

// tests
const char* ufr_gtw_webots_test_args(const link_t* link) {

}

static
ufr_gtw_api_t ufr_gtw_webots_api = {
	.type = ufr_gtw_webots_type,
	.state = ufr_gtw_webots_state,
	.size = ufr_gtw_webots_size,

	.boot = ufr_gtw_webots_boot,
	.start = ufr_gtw_webots_start,
	.stop = ufr_gtw_webots_stop,
	.copy = ufr_gtw_webots_copy,

	.read = ufr_gtw_webots_read,
	.write = ufr_gtw_webots_write,

	.recv = ufr_gtw_webots_recv,
	.recv_async = ufr_gtw_webots_recv_async,

    .accept = ufr_gtw_webots_accept,

    .test_args = ufr_gtw_webots_test_args,
};


// ============================================================================
//  Public Function
// ============================================================================

int ufr_gtw_webots_new(link_t* link, int type) {
    ufr_init_link(link, &ufr_gtw_webots_api);
    return UFR_OK;
}

int ufr_gtw_webots_get_time_step() {
    return g_time_step;
}