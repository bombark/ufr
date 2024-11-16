/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica Imagem (VRI)
 *  - Felipe Bombardelli
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h> 
#include <arpa/inet.h>

#include "ufr_zmq_common.h"

void* g_context = NULL;

// ============================================================================
//  Topic
// ============================================================================

int ufr_zmq_type(const link_t* link) {
    return 0;
}

int ufr_zmq_state(const link_t* link) {
    return 0;
}

size_t ufr_zmq_size(const link_t* link, int type) {
    ll_obj_t* local = link->gtw_shr;
    if ( local ) {
        return zmq_msg_size(&local->recv_msg);
    }
    return 0;
}

int ufr_zmq_boot (link_t* link, const ufr_args_t* args) {
    // get optional parameter context
    const char* host = ufr_args_gets(args, "@host", "127.0.0.1");

    // get optional parameter context
    void* context = (void*) ufr_args_getp(args, "@context", NULL);
    if ( context == NULL ) {
        if ( g_context == NULL ) {
            ufr_info(link, "creating zmq context");
            g_context = zmq_ctx_new();
        }
        context = g_context;
    }

    // get optional parameter port
    const uint32_t port = ufr_args_geti(args, "@port", 5000);

	// prepare shared data with the socket parameters
	const size_t host_str_len = strlen(host);
	ll_shr_t* shr = malloc( sizeof(ll_shr_t) + host_str_len + 1 );
    if ( shr == NULL ) {
        return ufr_error(link, ENOMEM, "%s", strerror(ENOMEM));
    }
	shr->context = context;
	shr->port = port;
    strcpy(shr->host, host);

    // prepare the gateway object
    ll_obj_t* obj = malloc( sizeof(ll_obj_t) );
    if ( obj == NULL ) {
        free(shr);
        return ufr_error(link, ENOMEM, "%s", strerror(ENOMEM));
    }
    obj->socket = NULL;
    zmq_msg_init(&obj->recv_msg);

    // update the link
    link->gtw_shr = shr;
    link->gtw_obj = obj;

    // success
    return UFR_OK;
}

void ufr_zmq_stop(link_t* link, int type) {
    if ( type == UFR_STOP_CLOSE ) {
        ll_obj_t* obj = link->gtw_obj;
        if ( obj ) {
            zmq_close(obj->socket);
            free(obj);
            link->gtw_obj = NULL;
        }
    }
}

int ufr_zmq_recv(link_t* link) {
    // read the message
    ll_obj_t* local = link->gtw_obj;
    const int msg_size = zmq_msg_recv (&local->recv_msg, local->socket, 0);
    if ( msg_size < 0 ) {
        return -1;
    }

    // start the index
    local->idx = 0;

    // link has decoder, call the dcr_api->recv()
    if ( link->dcr_api != NULL ) {
        uint8_t* recv_msg_data = zmq_msg_data(&local->recv_msg);
        const size_t recv_msg_size = zmq_msg_size(&local->recv_msg);
        link->dcr_api->recv_cb(link, (char*) recv_msg_data, recv_msg_size);
    }

    // success
    return UFR_OK;
}

int ufr_zmq_recv_async(link_t* link) {
    ll_obj_t* gtw_obj = link->gtw_obj;
    gtw_obj->idx = 0;

    const int msg_size = zmq_msg_recv (&gtw_obj->recv_msg, gtw_obj->socket, ZMQ_DONTWAIT);
    if ( msg_size < 0 ) {
        return -1;
    }

    if ( link->dcr_api != NULL ) {
        uint8_t* recv_msg_data = zmq_msg_data(&gtw_obj->recv_msg);
        const size_t recv_msg_size = zmq_msg_size(&gtw_obj->recv_msg);
        link->dcr_api->recv_cb(link, (char*) recv_msg_data, recv_msg_size);
    }

    return UFR_OK;
}


size_t ufr_zmq_read(link_t* link, char* buffer, size_t max_size) {
    ll_obj_t* local = link->gtw_obj;

    if ( local == NULL ) {
        return 0;
    }

    //
    const size_t offset = local->idx;
    const uint8_t* msg_data = zmq_msg_data(&local->recv_msg);
    const size_t msg_size = zmq_msg_size(&local->recv_msg);
    if ( offset > msg_size ) {
        return 0;
    }

    //
    size_t copied = 0;
    const size_t rest_of_msg = msg_size - offset;
    if ( rest_of_msg > max_size ) {
        strncpy(buffer, msg_data, max_size);
        local->idx += max_size;
        copied = max_size;
    } else {
        strncpy(buffer, msg_data, rest_of_msg);
        local->idx += rest_of_msg;
        copied = rest_of_msg;
    }

    // success
    return copied;
}

size_t ufr_zmq_write(link_t* link, const char* buffer, size_t size) {
    ll_obj_t* gtw_obj = link->gtw_obj;
    if ( gtw_obj == NULL ) {
        return 0;
    }

    // send the data to buffer
    // const size_t sent = zmq_send (local->socket, buffer, size, ZMQ_SNDMORE);
    const size_t sent = zmq_send (gtw_obj->socket, buffer, size, 0);
    if ( sent != size ) {
        return ufr_error(link, 0, "%s", zmq_strerror(errno));
    }
    ufr_info(link, "sent %ld bytes", sent);
    return sent;
}

int ufr_zmq_recv_peer_name(link_t* link, char* buffer, size_t maxbuffer) {
    ll_obj_t* gtw_obj = (ll_obj_t*) link->gtw_obj;
    const int sockfd = zmq_msg_get (&gtw_obj->recv_msg, ZMQ_SRCFD);
    struct sockaddr_in addr;
    socklen_t asize = sizeof(addr);
    getpeername(sockfd, (struct sockaddr*)&addr, &asize);
    char* ip = inet_ntoa(addr.sin_addr);
    strcpy(buffer, ip);
    return UFR_OK;
}