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
//  Socket
// ============================================================================

static
int ufr_zmq_socket_type(const link_t* link) {
    return UFR_TYPE_SOCKET;
}

static
int ufr_zmq_socket_start(link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_CONNECT ) {
        ufr_info(link, "creating a socket");
        const ll_shr_t* shr = link->gtw_shr;
        void* socket = zmq_socket (shr->context, ZMQ_REQ);
        if ( socket == NULL ) {
            return ufr_error(link, errno, "%s (context: %p)", zmq_strerror(errno), shr->context);
        }

        // connect
        char url[512];
        snprintf(url, sizeof(url), "tcp://%s:%d", shr->host, shr->port);
        ufr_info(link, "connecting to the URL %s", url);
        if ( zmq_connect (socket, url) != 0 ) {
            return ufr_error(link, errno, "%s", zmq_strerror(errno));
        }

        // update the gtw_obj
        ll_obj_t* obj = link->gtw_obj;
        obj->socket = socket;

    } else if ( type == UFR_START_BIND ) {
        //
        const ll_shr_t* shr_data = link->gtw_shr;
        void* socket = zmq_socket (shr_data->context, ZMQ_REP);
        if ( socket == NULL ) {
            return ufr_error(link, errno, "%s", zmq_strerror(errno));
        }

        //
        char url[512];
        snprintf(url, sizeof(url), "tcp://%s:%d", shr_data->host, shr_data->port);
        int error = zmq_bind (socket, url);
        if ( error != 0 ) {
            return ufr_error(link, errno, "%s", zmq_strerror(errno));
        }

        //
        ll_obj_t* local = link->gtw_obj;
        local->socket = socket;
    }

    return 0;
}

int ufr_zmq_socket_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

static
ufr_gtw_api_t ufr_zmq_socket_api = {
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
    .accept = ufr_zmq_socket_accept
};

int ufr_gtw_zmq_new_socket(link_t* link, int type) {
	ufr_init_link(link, &ufr_zmq_socket_api);
	return UFR_OK;
}
