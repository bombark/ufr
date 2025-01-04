/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica Imagem (VRI)
 *   Felipe Bombardelli
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
 * */
	
// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <zmq.h>
#include <ufr.h>

#include "ufr_zmq_common.h"

// ============================================================================
//  Topic
// ============================================================================

static
int ufr_zmq_topic_type(const link_t* link) {
    return UFR_TYPE_TOPIC;
}

static
int ufr_zmq_topic_start(struct _link* link, int type, const ufr_args_t* args) {
    // Publisher
	if ( type == UFR_START_PUBLISHER ) {
        // begin
        const ll_shr_t* obj_args = link->gtw_shr;

        // Socket
        ufr_log_ini(link, "starting socket");
        void* socket = zmq_socket(obj_args->context, ZMQ_PUB);
        if ( socket == NULL ) {
            return 1;
        }
        ufr_log_end(link, "socket started");

        // Bind
        ufr_log_ini(link, "starting binding");
        char url[512];
        snprintf(url, sizeof(url), "tcp://%s:%d", obj_args->host, obj_args->port);
        if ( zmq_bind(socket, url) != 0 ) {
            return 1;
        }
        ufr_log_end(link, "binded publisher on %s binding the port %d", obj_args->host, obj_args->port);

        // Update object 
        ufr_info(link, "update gtw_obj");
        ll_obj_t* local = link->gtw_obj;
        local->socket = socket;

        // wait for router : Pieter's documentation calls the "slow joiner"
        // https://stackoverflow.com/questions/11634830/zeromq-always-loses-the-first-message
        ufr_info(link, "wait 1 sec for slow joiner");
        sleep(1);

    // Subscriber
    } else if ( type == UFR_START_SUBSCRIBER ) {
        // begin
        const ll_shr_t* obj_args = link->gtw_shr;

        // socket
        ufr_log_ini(link, "starting socket");
        void* socket = zmq_socket(obj_args->context, ZMQ_SUB);
        if ( socket == NULL ) {
            return 1;
        }
        ufr_log_end(link, "socket started");

        // connection
        ufr_log_ini(link, "connecting the socket");
        char url[512];
        snprintf(url, sizeof(url), "tcp://%s:%d", obj_args->host, obj_args->port);
        if ( zmq_connect(socket, url) != 0 ) {
            return 1;
        }
        zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);
        ufr_log_end(link, "connected subscriber on %s connecting with the port %d", obj_args->host, obj_args->port);

        // update object
        ufr_info(link, "update gtw_obj");
        ll_obj_t* local = link->gtw_obj;
        local->socket = socket;

    // Other types
    } else {
        // error
        return ufr_error(link, 1, "start type invalid (%d)", type);
    }

    return 0;
}

static
ufr_gtw_api_t ufr_zmq_topic_api = {
    .name = "zmq/topic",
	.type = ufr_zmq_topic_type,
	.state = ufr_zmq_state,
	.size = ufr_zmq_size,
	.boot = ufr_zmq_boot,
	.start = ufr_zmq_topic_start,
	.stop = ufr_zmq_stop,
	.copy = NULL,
	.recv = ufr_zmq_recv,
	.recv_async = ufr_zmq_recv_async,
	.read = ufr_zmq_read,
	.write = ufr_zmq_write,
    .recv_peer_name = ufr_zmq_recv_peer_name
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_zmq_new_topic(link_t* link) {
    ufr_init_link(link, &ufr_zmq_topic_api);
	return UFR_OK;
}
