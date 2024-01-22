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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <netdb.h> /* getprotobyname */
#include <netinet/in.h>
#include <sys/socket.h>
#include <ufr.h>

typedef struct {
    int fd_read;
    int server_sockfd;
} ll_shr_t;

// ============================================================================
//  Socket Driver
// ============================================================================

static
int lt_posix_socket_type(const link_t* link) {
	return 0;
}

static
int lt_posix_socket_state(const link_t* link){
	return 0;
}

static
size_t lt_posix_socket_size(const link_t* link, int type){
	return 0;
}

static
int lt_posix_socket_boot(link_t* link, const lt_args_t* args) {
    ll_shr_t* shr = malloc(sizeof(ll_shr_t));
    shr->server_sockfd = 0;
    link->gw_shr = shr;


	return 0;
}

static
int lt_posix_socket_start(link_t* link, int type, const lt_args_t* args) {

    if ( type == LT_START_BIND ) {

        struct protoent *protoent;

        protoent = getprotobyname("tcp");
        if (protoent == NULL) {
            perror("getprotobyname");
            exit(EXIT_FAILURE);
        }

        int server_sockfd = socket(AF_INET, SOCK_STREAM, protoent->p_proto);
        if (server_sockfd == -1) {
            perror("socket");
            exit(EXIT_FAILURE);
        }

        int enable = 1;
        if (setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
            perror("setsockopt(SO_REUSEADDR) failed");
            exit(EXIT_FAILURE);
        }

        struct sockaddr_in server_address;
        server_address.sin_family = AF_INET;
        server_address.sin_addr.s_addr = htonl(INADDR_ANY);
        server_address.sin_port = htons(2000);
        int error = bind( server_sockfd, (struct sockaddr*)&server_address, sizeof(server_address) );
        if ( error == -1 ) {
            perror("bind");
            exit(EXIT_FAILURE);
        }

        if (listen(server_sockfd, 5) == -1) {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        ll_shr_t* shr = (ll_shr_t*) link->gw_shr;
        shr->server_sockfd = server_sockfd;
    }

	return 0;
}

static
void lt_posix_socket_stop(link_t* link, int type) {
    ll_shr_t* shr = link->gw_shr;
    if ( shr != NULL ) {

    }
}

static
int lt_posix_socket_copy(link_t* link, link_t* out) {
    out->gw_shr = link->gw_shr;
	return 0;
}

static
size_t lt_posix_socket_read(link_t* link, char* buffer, size_t length) {
	ll_shr_t* shr = link->gw_shr;
	return read(shr->server_sockfd, buffer, length);
}

static
size_t lt_posix_socket_write(link_t* link, const char* buffer, size_t length) {
	ll_shr_t* shr = link->gw_shr;
    return 0;
}

static
void lt_posix_socket_recv(link_t* link) {
    ll_shr_t* shr = link->gw_shr;
    // int client_sockfd = accept(shr->server_sockfd, (struct sockaddr*)&client_address, &client_len);
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
	.read = lt_posix_socket_read,
	.write = lt_posix_socket_write,
    .recv = lt_posix_socket_recv
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_new_gtw_posix_socket(link_t* link, const lt_args_t* args) {
	link->gw_api = &lt_posix_socket;
	lt_posix_socket_boot(link, args);
	return LT_OK;
}
