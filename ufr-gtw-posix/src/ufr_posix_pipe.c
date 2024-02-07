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
#include <ufr.h>

#define PIPE_READ 0
#define PIPE_WRTE 1
#define BUFFER_SIZE 1024

typedef struct {
    int fd_read;
    int fd_wrte;
} ll_shr_t;

// ============================================================================
//  File
// ============================================================================

static
int lt_posix_pipe_type(const link_t* link) {
	return 0;
}

static
int lt_posix_pipe_state(const link_t* link){
	return 0;
}

static
size_t lt_posix_pipe_size(const link_t* link, int type){
	return 0;
}

static
int lt_posix_pipe_boot(link_t* link, const lt_args_t* args) {
    ll_shr_t* shr = malloc(sizeof(ll_shr_t));

    if ( pipe( (int*)shr ) < 0 ) {
        return 1;
    }
	
    link->gw_shr = shr;
    link->gw_obj = malloc(BUFFER_SIZE);

	return 0;
}

static
int lt_posix_pipe_start(link_t* link, int type, const lt_args_t* args) {
	return 0;
}

static
void lt_posix_pipe_stop(link_t* link, int type) {
    ll_shr_t* shr = link->gw_shr;
    if ( shr != NULL ) {
        close(shr->fd_read);
        close(shr->fd_wrte);
        free(shr);
    }
}

static
int lt_posix_pipe_copy(link_t* link, link_t* out) {
    out->gw_shr = link->gw_shr;
	return 0;
}

static
size_t lt_posix_pipe_read(link_t* link, char* buffer, size_t length) {
	ll_shr_t* shr = link->gw_shr;
	return read(shr->fd_read, buffer, length);
}

static
size_t lt_posix_pipe_write(link_t* link, const char* buffer, size_t length) {
	ll_shr_t* shr = link->gw_shr;
    return write(shr->fd_wrte, buffer, length);
}

static
bool lt_posix_pipe_recv(link_t* link) {
    char* msg_data = (char*) link->gw_obj;
    ll_shr_t* shr = (ll_shr_t*) link->gw_shr;
    
    // read one line
    size_t msg_size = 0;
    while (msg_size<(BUFFER_SIZE-1) ) {
        char c;
        const int nbytes = read(shr->fd_read, &c, 1);
        if ( nbytes != 1 ) {
            break;
        }
        msg_data[msg_size] = c;
        msg_size += 1;
        if ( c == '\n' ) {
            break;
        }
    }
    msg_data[msg_size] = '\0';

    if ( link->dec_api != NULL ){
        link->dec_api->recv(link, msg_data, msg_size);
    }

    return true;
}

static
lt_api_t lt_posix_pipe = {
	.type = lt_posix_pipe_type,
	.state = lt_posix_pipe_state,
	.size = lt_posix_pipe_size,
	.boot = lt_posix_pipe_boot,
	.start = lt_posix_pipe_start,
	.stop = lt_posix_pipe_stop,
	.copy = lt_posix_pipe_copy,
	.read = lt_posix_pipe_read,
	.write = lt_posix_pipe_write,
    .recv = lt_posix_pipe_recv
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_posix_new_pipe(link_t* link, int type) {
	link->gtw_api = &lt_posix_pipe;
    link->type_started = type;
    return LT_OK;
}
