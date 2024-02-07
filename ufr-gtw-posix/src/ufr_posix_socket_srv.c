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
#include "ufr_message.h"

typedef struct {
    int lenght;
    struct sockaddr address;
    int sockfd;
    message_t message;
} ll_srv_request_t;

// ============================================================================
//  Server Driver
// ============================================================================

static
int ufr_posix_socket_start_server(link_t* link, int type, const lt_args_t* args) {
    // get the parameters for the server
    const uint16_t port = 2000;

    // get TCP protocol
    struct protoent *protoent;
    protoent = getprotobyname("tcp");
    if (protoent == NULL) {
        return lt_error(link, 1, "getprotobyname");
    }

    // start the socket
    int server_sockfd = socket(AF_INET, SOCK_STREAM, protoent->p_proto);
    if (server_sockfd == -1) {
        return lt_error(link, 1, "error to open socket");
    }

    // configure the socket
    int enable = 1;
    if (setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        close(server_sockfd);
        return lt_error(link, 1, "setsockopt(SO_REUSEADDR) failed");
    }

    // bind the socket
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_address.sin_port = htons(port);
    int error = bind( server_sockfd, (struct sockaddr*)&server_address, sizeof(server_address) );
    if ( error == -1 ) {
        perror("bind");
        return lt_error(link, 1, "error to bind the port");
    }

    // listen the socket
    if (listen(server_sockfd, 5) == -1) {
        perror("listen");
        return lt_error(link, 1, "error to listen the socket");
    }

    // update the shared object
    ll_shr_t* shr = (ll_shr_t*) link->gw_shr;
    shr->server_sockfd = server_sockfd;
    link->gw_obj = NULL;

    // success
    return LT_OK;
}

static
void lt_posix_socket_srv_stop(link_t* link, int type) {
    ll_srv_request_t* request = link->gw_obj;
    if ( request != NULL ) {
        if ( request->sockfd > 0 ) {
            close(request->sockfd);
        }
        request->sockfd = 0;
    }
}

static
size_t lt_posix_socket_srv_read(link_t* link, char* buffer, size_t length) {
	ll_shr_t* shr = link->gw_shr;
	return read(shr->server_sockfd, buffer, length);
}

static
size_t lt_posix_socket_srv_write(link_t* link, const char* buffer, size_t length) {
	ll_srv_request_t* request = link->gw_obj;
    return write(request->sockfd, buffer, length);
}

static
bool lt_posix_socket_srv_recv(link_t* link) {
    if ( link->gw_obj == NULL ) {
        ll_srv_request_t* request = malloc(sizeof(ll_srv_request_t));
        message_init(&request->message);
        link->gw_obj = request;
    }

    ll_srv_request_t* request = link->gw_obj;
    ll_shr_t* shr = link->gw_shr;
    
    request->sockfd = accept(shr->server_sockfd, &request->address, &request->lenght);
    message_write_from_fd(&request->message, request->sockfd);
    return true;
}

static
int lt_posix_socket_srv_send(struct _link* link) {
    ll_srv_request_t* request = link->gw_obj;
    if ( request != NULL && request->sockfd > 0 ) {
        close(request->sockfd);
        request->sockfd = 0;
    }
}

lt_api_t ufr_posix_socket_srv = {
	.type = lt_posix_socket_type,
	.state = lt_posix_socket_state,
	.size = lt_posix_socket_size,
	.boot = lt_posix_socket_boot,
	.start = ufr_posix_socket_start_server,
	.stop = lt_posix_socket_srv_stop,
	.copy = lt_posix_socket_copy,
	.read = lt_posix_socket_srv_read,
	.write = lt_posix_socket_srv_write,
    .recv = lt_posix_socket_srv_recv,
    .send = lt_posix_socket_srv_send
};
