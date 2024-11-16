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
#include <string.h>
#include <ufr.h>
#include <unistd.h>

#define PIPE_READ 0
#define PIPE_WRTE 1
#define BUFFER_SIZE 1024

typedef struct {
    int fd_read;
    int fd_wrte;
} ll_shr_t;

typedef struct {
    uint32_t idx;
    uint32_t size;
    uint32_t maxsize;
    uint8_t* data;
} recv_buffer_t;

const uint32_t g_ass = 0xCBFBAABB;

// ============================================================================
//  File
// ============================================================================

static
int ufr_posix_pipe_type(const link_t* link) {
	return 0;
}

static
int ufr_posix_pipe_state(const link_t* link){
	return 0;
}

static
size_t ufr_posix_pipe_size(const link_t* link, int type){
	return 0;
}

static
int ufr_posix_pipe_boot(link_t* link, const ufr_args_t* args) {
    ll_shr_t* shr = malloc(sizeof(ll_shr_t));

    if ( pipe( (int*)shr ) < 0 ) {
        return 1;
    }
	
    link->gtw_shr = shr;

    recv_buffer_t* buffer = malloc( sizeof(recv_buffer_t) );
    if ( buffer == NULL ) {
        ufr_fatal(link, 1, "Memory fault");
    }

    buffer->idx = 0;
    buffer->size = 0;
    buffer->maxsize = BUFFER_SIZE;
    buffer->data = malloc(BUFFER_SIZE);
    link->gtw_obj = buffer;

	return 0;
}

static
int ufr_posix_pipe_start(link_t* link, int type, const ufr_args_t* args) {
	return 0;
}

static
void ufr_posix_pipe_stop(link_t* link, int type) {
    ll_shr_t* shr = link->gtw_shr;
    if ( shr != NULL ) {
        close(shr->fd_read);
        close(shr->fd_wrte);
        free(shr);
    }
}

static
int ufr_posix_pipe_copy(link_t* link, link_t* out) {
    out->gtw_shr = link->gtw_shr;
	return 0;
}

static
size_t ufr_posix_pipe_read(link_t* link, char* out_buffer, size_t length) {
	recv_buffer_t* buffer = (recv_buffer_t*) link->gtw_obj;
    const uint32_t idx = buffer->idx;
    if ( idx >= buffer->size ) {
        return 0;
    }

    const uint32_t rest_size =  (buffer->size - idx);
    const uint32_t copied = (length < rest_size) ? length : rest_size;
    memcpy(out_buffer, &buffer->data[idx], copied);
    buffer->idx += copied;
    return copied;
}

static
size_t ufr_posix_pipe_write(link_t* link, const char* buffer, size_t length) {
    ll_shr_t* shr = link->gtw_shr;
    
    const size_t retval1 = write(shr->fd_wrte, &g_ass, sizeof(g_ass));
    if ( retval1 != sizeof(g_ass) ) {
        return 0;
    }

    uint32_t ui32_length = length;
    const size_t retval2 = write(shr->fd_wrte, &ui32_length, sizeof(ui32_length));
    if ( retval2 != sizeof(ui32_length) ) {
        return 0;
    }
    const size_t retval3 = write(shr->fd_wrte, buffer, length);
    return retval3;
}

static
int ufr_posix_pipe_recv(link_t* link) {
    recv_buffer_t* buffer = (recv_buffer_t*) link->gtw_obj;
    ll_shr_t* shr = (ll_shr_t*) link->gtw_shr;
    
    uint32_t ass;
    const uint32_t ret0 = read(shr->fd_read, &ass, sizeof(ass));
    if ( ret0 != sizeof(ass) || ass != g_ass ) {
        return -1;
    }

    uint32_t size;
    const uint32_t ret1 = read(shr->fd_read, &size, sizeof(size));
    if ( ret1 != sizeof(ret1) ) {
        return -1;
    }

    // realloc the buffer, case needed
    if ( buffer->maxsize < size ) {
        if ( buffer->data ) {
            free(buffer->data);
        }
        uint8_t* new_data = malloc(size);
        if ( new_data == NULL ) {
            ufr_fatal(link, 1, "Memory out");
        }
        buffer->data = new_data;
        buffer->maxsize = size;
    }

    // read from pipe
    const uint32_t msg_size = read(shr->fd_read, buffer->data, size);
    buffer->idx = 0;
    buffer->size = msg_size;
    if ( link->dcr_api != NULL ) {
        if ( link->dcr_api->recv_cb != NULL ) {
            link->dcr_api->recv_cb(link, buffer->data, msg_size);
        } else {
            ufr_info(link, "Function recv_cb on Decoder API is NULL");
        }
    } else {
        ufr_info(link, "Decoder API is NULL");
    }

    // success
    return UFR_OK;
}

static
ufr_gtw_api_t ufr_posix_pipe = {
	.type = ufr_posix_pipe_type,
	.state = ufr_posix_pipe_state,
	.size = ufr_posix_pipe_size,
	.boot = ufr_posix_pipe_boot,
	.start = ufr_posix_pipe_start,
	.stop = ufr_posix_pipe_stop,
	.copy = ufr_posix_pipe_copy,
	.read = ufr_posix_pipe_read,
	.write = ufr_posix_pipe_write,
    .recv = ufr_posix_pipe_recv
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_posix_new_pipe(link_t* link, int type) {
	// link->gtw_api = &ufr_posix_pipe;
    ufr_init_link(link, &ufr_posix_pipe);
    return UFR_OK;
}
