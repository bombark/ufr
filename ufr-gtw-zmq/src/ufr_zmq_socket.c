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
#include <zmq.h>
#include <ufr.h>

#include "ufr_zmq_common.h"

// ============================================================================
//  Socket Commom Functions
// ============================================================================

static
int ufr_zmq_socket_type(const link_t* link) {
    return UFR_TYPE_SOCKET;
}

static
int ufr_zmq_socket_start(link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_CLIENT ) {
        ufr_log_ini(link, "creating a socket");
        const ll_shr_t* shr = link->gtw_shr;
        void* socket = zmq_socket (shr->context, ZMQ_REQ);
        if ( socket == NULL ) {
            return ufr_log_error(link, errno, "%s (context: %p)", zmq_strerror(errno), shr->context);
        }
        ufr_log_end(link, "socket created");

        // connect
        ufr_log_ini(link, "connecting to the server");
        char url[512];
        snprintf(url, sizeof(url), "tcp://%s:%d", shr->host, shr->port);
        if ( zmq_connect (socket, url) != 0 ) {
            return ufr_log_error(link, errno, "%s", zmq_strerror(errno));
        }
        ufr_log_end(link, "connected to the server at URL %s", url);
        
        // update the gtw_obj
        ufr_log_ini(link, "updating the gateway object");
        ll_obj_t* obj = link->gtw_obj;
        obj->socket = socket;
        ufr_log_end(link, "gateway object updated");

        // set timeout
        int time = 2000;
        zmq_setsockopt(socket, ZMQ_RCVTIMEO, &time, sizeof(time));

    } else if ( type == UFR_START_SERVER_ST || type == UFR_START_SERVER_MT ) {
        // create the socket
        ufr_log_ini(link, "creating the socket");
        const ll_shr_t* shr_data = link->gtw_shr;
        void* socket = zmq_socket (shr_data->context, ZMQ_REP);
        if ( socket == NULL ) {
            return ufr_log_error(link, errno, "%s", zmq_strerror(errno));
        }
        ufr_log_end(link, "socket created");

        // bind the port
        ufr_log_ini(link, "binding the port");
        char url[512];
        snprintf(url, sizeof(url), "tcp://%s:%d", shr_data->host, shr_data->port);
        int error = zmq_bind (socket, url);
        if ( error != 0 ) {
            return ufr_log_error(link, errno, "%s", zmq_strerror(errno));
        }
        ufr_log_end(link, "port binded %s", url);

        // update the gtw_obj
        ufr_log_ini(link, "updating the gateway object");
        ll_obj_t* local = link->gtw_obj;
        local->socket = socket;
        ufr_log_end(link, "gateway object updated");
    }

    return 0;
}

// ============================================================================
//  Socket Single Thread
// ============================================================================

static
ufr_gtw_api_t ufr_zmq_socket_st_api = {
    .name = "zmq",
	.type = ufr_zmq_socket_type,
	.state = ufr_zmq_state,
	.size = ufr_zmq_size,
	.boot = ufr_zmq_boot,
	.start = ufr_zmq_socket_start,
	.stop = ufr_zmq_stop,
	.copy = NULL,
    .recv = ufr_zmq_recv,
    .recv_async = ufr_zmq_recv_async,
	.read = ufr_zmq_read,
	.write = ufr_zmq_write,
    .accept = NULL,
    .recv_peer_name = ufr_zmq_recv_peer_name
};

// ============================================================================
//  Socket Multi Thread
// ============================================================================

size_t ufr_zmq_socket_write(link_t* link, const char* buffer, size_t size) {
    ll_obj_t* gtw_obj = link->gtw_obj;
    if ( gtw_obj == NULL ) {
        return 0;
    }

    // send the data to buffer
    size_t sent;
    if ( size > 0 ) {
        sent = zmq_send (gtw_obj->socket, buffer, size, ZMQ_SNDMORE);
    } else {
        sent = zmq_send (gtw_obj->socket, NULL, 0, 0); 
    }

    // const size_t sent = zmq_send (gtw_obj->socket, buffer, size, 0);
    if ( sent != size ) {
        return ufr_error(link, 0, "%s", zmq_strerror(errno));
    }
    ufr_info(link, "sent %ld bytes", sent);
    return sent;
}

int ufr_zmq_socket_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

static
ufr_gtw_api_t ufr_zmq_socket_mt_api = {
    .name = "zmq",
	.type = ufr_zmq_socket_type,
	.state = ufr_zmq_state,
	.size = ufr_zmq_size,
	.boot = ufr_zmq_boot,
	.start = ufr_zmq_socket_start,
	.stop = ufr_zmq_stop,
	.copy = NULL,
    .recv = ufr_zmq_recv,
    .recv_async = ufr_zmq_recv_async,
	.read = ufr_zmq_read,
	.write = ufr_zmq_socket_write,
    .accept = ufr_zmq_socket_accept,
    .recv_peer_name = ufr_zmq_recv_peer_name
};

// ============================================================================
//  Public
// ============================================================================

int ufr_gtw_zmq_new_socket(link_t* link, int type) {
    if ( type == UFR_START_SERVER_ST || type == UFR_START_CLIENT ) {
	    ufr_init_link(link, &ufr_zmq_socket_st_api);
    } else if ( type == UFR_START_SERVER_MT ) {
        ufr_init_link(link, &ufr_zmq_socket_mt_api);
    } else {
        return ufr_error(link, 1, "invalid type");
    }
	return UFR_OK;
}
