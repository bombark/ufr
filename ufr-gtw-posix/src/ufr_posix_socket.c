/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Felipe Bombardelli
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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>


#include <netdb.h> /* getprotobyname */
#include <netinet/in.h>
#include <sys/socket.h>
#include <ufr.h>

#include "ufr_posix_socket.h"

typedef struct {
    size_t size;
    size_t max;
    char* ptr;
} message_t;

typedef struct {
    int lenght;
    struct sockaddr address;
    int sockfd;
    message_t message;
} ll_srv_request_t;

// ============================================================================
//  Socket Driver
// ============================================================================

int lt_posix_socket_type(const link_t* link) {
	return 0;
}

int lt_posix_socket_state(const link_t* link){
	return 0;
}

size_t lt_posix_socket_size(const link_t* link, int type){
	return 0;
}

int lt_posix_socket_boot(link_t* link, const lt_args_t* args) {
    ll_shr_t* shr = malloc(sizeof(ll_shr_t));
    shr->server_sockfd = 0;
    link->gw_shr = shr;
	return 0;
}

int lt_posix_socket_start(link_t* link, int type, const lt_args_t* args) {
    int retval = 1;
    // Start a Server
    if ( type == LT_START_BIND ) {
        retval = ufr_posix_start_server(link, args);
    // Start a Client
    } else if ( type == LT_START_CONNECT ) {
        retval = ufr_posix_start_client(link, args);
    }
	return retval;
}

void lt_posix_socket_stop(link_t* link, int type) {
    ll_srv_request_t* request = link->gw_obj;
    if ( request != NULL ) {
        if ( request->sockfd > 0 ) {
            close(request->sockfd);
        }
        request->sockfd = 0;
    }
}

int lt_posix_socket_copy(link_t* link, link_t* out) {
    out->gw_shr = link->gw_shr;
	return 0;
}

static
lt_api_t lt_posix_socket = {
	.type = lt_posix_socket_type,
	.state = lt_posix_socket_state,
	.size = lt_posix_socket_size,
	.boot = lt_posix_socket_boot,
	.start = lt_posix_socket_start,
	.stop = lt_posix_socket_stop,
	.copy = lt_posix_socket_copy,
	.read = ufr_dummy_read,
	.write = ufr_dummy_write,
    .recv = ufr_dummy_recv,
    .send = ufr_dummy_send
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_new_gtw_posix_socket(link_t* link, const lt_args_t* args) {
	link->gw_api = &lt_posix_socket;
	lt_posix_socket_boot(link, args);
	return LT_OK;
}
