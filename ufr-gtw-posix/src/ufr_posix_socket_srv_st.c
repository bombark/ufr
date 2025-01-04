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
int ufr_posix_socket_start_server(link_t* link, int type, const ufr_args_t* args) {
link->log_level = 10;
    // get the parameters for the server
    const uint16_t port = ufr_args_geti(args, "@port", 2000);

    // get TCP protocol
    ufr_log_ini(link, "get TCP protocol");
    struct protoent *protoent;
    protoent = getprotobyname("tcp");
    if (protoent == NULL) {
        return ufr_error(link, 1, "getprotobyname");
    }
    ufr_log_end(link, "get TCP protocol");

    // start the socket
    ufr_log_ini(link, "start the socket");
    int server_sockfd = socket(AF_INET, SOCK_STREAM, protoent->p_proto);
    if (server_sockfd == -1) {
        return ufr_error(link, 1, "error to open socket");
    }
    ufr_log_end(link, "socket started");

    // configure the socket
    int enable = 1;
    if (setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        close(server_sockfd);
        return ufr_error(link, 1, "setsockopt(SO_REUSEADDR) failed");
    }

    // bind the socket
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_address.sin_port = htons(port);
    int error = bind( server_sockfd, (struct sockaddr*)&server_address, sizeof(server_address) );
    if ( error == -1 ) {
        perror("bind");
        return ufr_error(link, 1, "error to bind the port");
    }

    // listen the socket
    if (listen(server_sockfd, 5) == -1) {
        perror("listen");
        return ufr_error(link, 1, "error to listen the socket");
    }

    // update the shared object
    ll_shr_t* shr = (ll_shr_t*) link->gtw_shr;
    shr->server_sockfd = server_sockfd;
    link->gtw_obj = NULL;

    // success
    ufr_log(link, "server listen on %d", port);
    return UFR_OK;
}

static
void ufr_posix_socket_srv_stop(link_t* link, int type) {
    ll_srv_request_t* request = link->gtw_obj;
    if ( request != NULL ) {
        if ( request->sockfd > 0 ) {
            close(request->sockfd);
        }
        request->sockfd = 0;
    }
}

static
size_t ufr_posix_socket_srv_read(link_t* link, char* buffer, size_t length) {
	ll_srv_request_t* request = link->gtw_obj;
	// return read(request->message.ptr, buffer, length);

    return 0;
}

static
size_t ufr_posix_socket_srv_write(link_t* link, const char* buffer, size_t length) {
	ll_srv_request_t* request = link->gtw_obj;
    size_t wrote = 0;
    if ( length == 0 ) {
        close(request->sockfd);
    } else {
        wrote = send(request->sockfd, buffer, length, 0);
    }
    return wrote;
}

static
int ufr_posix_socket_srv_recv(link_t* link) {
    if ( link->gtw_obj == NULL ) {
        ll_srv_request_t* request = malloc(sizeof(ll_srv_request_t));
        message_init(&request->message);
        link->gtw_obj = request;
    }

    ll_srv_request_t* request = link->gtw_obj;
    ll_shr_t* shr = link->gtw_shr;
    request->sockfd = accept(shr->server_sockfd, &request->address, &request->lenght);
    message_write_from_fd(&request->message, request->sockfd);

    if ( link->dcr_api != NULL ) {
        link->dcr_api->recv_cb(link, request->message.ptr, request->message.size);
    }

    return UFR_OK;
}

ufr_gtw_api_t ufr_posix_socket_srv = {
    .name = "PosixSocketServerSt",
	.type = ufr_posix_socket_type,
	.state = ufr_posix_socket_state,
	.size = ufr_posix_socket_size,
	.boot = ufr_posix_socket_boot,
	.start = ufr_posix_socket_start_server,
	.stop = ufr_posix_socket_srv_stop,
	.copy = ufr_posix_socket_copy,
	.read = ufr_posix_socket_srv_read,
	.write = ufr_posix_socket_srv_write,
    .recv = ufr_posix_socket_srv_recv,
};
