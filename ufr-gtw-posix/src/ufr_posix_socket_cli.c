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
    int sockfd;
    message_t message;
} ll_conn_t;

// ============================================================================
//  Client Driver
// ============================================================================

static
void lt_posix_socket_cli_stop(link_t* link, int type) {
    
}

static
size_t lt_posix_socket_cli_read(link_t* link, char* buffer, size_t length) {
    ll_conn_t* conn = link->gw_obj;
    return read(conn->sockfd, buffer, length);
}

static
size_t lt_posix_socket_cli_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
bool lt_posix_socket_cli_recv(link_t* link) {
    return false;
}

static
int lt_posix_socket_cli_send(struct _link* link) {
    return 0;
}

lt_api_t ufr_posix_socket_cli = {
	.type = lt_posix_socket_type,
	.state = lt_posix_socket_state,
	.size = lt_posix_socket_size,
	.boot = lt_posix_socket_boot,
	.start = lt_posix_socket_start,
	.stop = lt_posix_socket_cli_stop,
	.copy = lt_posix_socket_copy,
	.read = lt_posix_socket_cli_read,
	.write = lt_posix_socket_cli_write,
    .recv = lt_posix_socket_cli_recv,
    .send = lt_posix_socket_cli_send
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_posix_start_client(link_t* link, const lt_args_t* args) {
    struct sockaddr_in serverAddr;
    socklen_t addr_size;

    // get the parameters
    const char* address = "127.0.0.1";
    const uint16_t port = 2000;

    /*---- Create the socket. The three arguments are: ----*/
    /* 1) Internet domain 2) Stream socket 3) Default protocol (TCP in this case) */
    const int sockfd = socket(PF_INET, SOCK_STREAM, 0);

    /*---- Configure settings of the server address struct ----*/
    /* Address family = Internet */
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(address);
    memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);  
    connect(sockfd, (struct sockaddr *) &serverAddr, addr_size);

    // update the link
    ll_conn_t* conn = malloc( sizeof(ll_conn_t) );
    conn->sockfd = sockfd;
    link->gw_obj = conn;

    // change the API function to client
    link->gw_api = &ufr_posix_socket_cli;
    return LT_OK;
}