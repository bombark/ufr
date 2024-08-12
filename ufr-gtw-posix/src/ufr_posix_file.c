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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <ufr.h>

#define RECV_BUFFER_SIZE_DEFAULT 1024

typedef struct {
    int count;
    int mode;
    char path[];
} ll_shr_t;

typedef struct {
    FILE* fd;
    size_t recv_buffer_size;
    char recv_buffer[];
} ll_gtw_obj_t;

// ============================================================================
//  Private Funtions
// ============================================================================

static
ll_gtw_obj_t* new_gtw_obj(FILE* fd, size_t recv_buffer_size) {
    ll_gtw_obj_t* gtw_obj = malloc( sizeof(ll_gtw_obj_t) + recv_buffer_size );
    assert( gtw_obj != NULL );
    gtw_obj->fd = fd;
    gtw_obj->recv_buffer_size = recv_buffer_size;
    return gtw_obj;
}

// ============================================================================
//  File
// ============================================================================

static
int ufr_gtw_posix_file_type(const link_t* link) {
	return 0;
}

static
int ufr_gtw_posix_file_state(const link_t* link){
	return 0;
}

static
size_t ufr_gtw_posix_file_size(const link_t* link, int type){
	return 0;
}

static
int ufr_gtw_posix_file_boot(link_t* link, const ufr_args_t* args) {
	const char* path = ufr_args_gets(args, "@path", NULL);
    if ( path == NULL ) {
        ufr_error(link, EINVAL, "Parameter @path is blank");
        return EINVAL;
    }

    // create shared data
    const size_t size = strlen(path);
    ll_shr_t* shr = malloc( sizeof(ll_shr_t) + size + 1 );
    shr->count = 1;
    strncpy(shr->path, path, size);
    shr->path[size] = '\0';
    link->gtw_shr = shr;

    // start the link, case mode is present
	const char* mode = ufr_args_gets(args, "@mode", NULL);
    if ( mode != NULL ) {
        FILE* fd = fopen(shr->path, mode);
        if ( fd == NULL ) {
            ufr_error(link, EINVAL, "File not found");
            return EINVAL;
        }
        link->gtw_obj = new_gtw_obj(fd, RECV_BUFFER_SIZE_DEFAULT);
    }

    // success
	return UFR_OK;
}

static
int ufr_gtw_posix_file_start(link_t* link, int type, const ufr_args_t* args) {
    // start publisher
    if ( type == UFR_START_PUBLISHER ) {
        ll_shr_t* shr = (ll_shr_t*) link->gtw_shr;
        FILE* fd = fopen(shr->path, "w");
        if ( fd == NULL ) {
            return ufr_error(link, errno, strerror(errno));
        }
        link->gtw_obj = new_gtw_obj(fd, RECV_BUFFER_SIZE_DEFAULT);

    // start subscriber
    } else if ( type == UFR_START_SUBSCRIBER ) {
        ll_shr_t* shr = (ll_shr_t*) link->gtw_shr;
        FILE* fd = fopen(shr->path, "r");
        if ( fd == NULL ) {
            return ufr_error(link, errno, strerror(errno));
        }
        link->gtw_obj = new_gtw_obj(fd, RECV_BUFFER_SIZE_DEFAULT);

    // error
    } else {
        return ufr_error(link, 1, "parameter type(%d) is invalid", type);
    }

    // success
	return UFR_OK;
}

static
void ufr_gtw_posix_file_stop(link_t* link, int type) {
    if ( link->gtw_obj != NULL ) {
        ll_gtw_obj_t* gtw_obj = link->gtw_obj;
        fclose(gtw_obj->fd);
        free(gtw_obj);
        link->gtw_obj = NULL;
    }

    if ( link->gtw_shr != NULL ) {
        ll_shr_t* shr = (ll_shr_t*) link->gtw_shr;
        if ( shr->count <= 1 ) {
            shr->count = 0;
            free(shr);
            link->gtw_shr = NULL;
        } else {
            shr->count -= 1;
        }
    }
}

static 
int ufr_gtw_posix_file_recv(link_t* link) {
    ll_gtw_obj_t* gtw_obj = link->gtw_obj;
    const char* buf = fgets(gtw_obj->recv_buffer, gtw_obj->recv_buffer_size, gtw_obj->fd);
    if ( buf == NULL ) {
        return -1;
    }

    if ( link->dcr_api != NULL ) {
        const size_t size_recv = strlen(gtw_obj->recv_buffer);
        link->dcr_api->recv(link, gtw_obj->recv_buffer, size_recv);
    }

    return UFR_OK;
}

static
int ufr_gtw_posix_file_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t ufr_gtw_posix_file_read(link_t* link, char* buffer, size_t length) {
	ll_gtw_obj_t* gtw_obj = link->gtw_obj;
    if ( gtw_obj == NULL ) {
        ufr_error(link, 1, "gtw_obj is null");
        return 0;
    }
	const size_t nbytes = fread(buffer, 1, length, gtw_obj->fd);
    ufr_info(link, "read %ld bytes", nbytes);
    return nbytes;
}

static
size_t ufr_gtw_posix_file_write(link_t* link, const char* buffer, size_t length) {
	ll_gtw_obj_t* gtw_obj = link->gtw_obj;
    if ( gtw_obj == NULL ) {
        ufr_error(link, 1, "gtw_obj is null");
        return 0;
    }
    if ( gtw_obj->fd == NULL ) {
        ufr_error(link, 1, "gtw_obj->fd is null");
        return 0;
    }
    const size_t nbytes = fwrite(buffer, 1, length, gtw_obj->fd);
    ufr_info(link, "wrote %ld bytes", nbytes);
    return nbytes;
}

const char* ufr_gtw_posix_file_test_args(const link_t* link) {
    return "@path teste.txt";
}

static
ufr_gtw_api_t ufr_gtw_posix_file_api = {
    .name = "file",
    .type = ufr_gtw_posix_file_type,
    .state = ufr_gtw_posix_file_state,
    .size = ufr_gtw_posix_file_size,
    .boot = ufr_gtw_posix_file_boot,
    .start = ufr_gtw_posix_file_start,
    .stop = ufr_gtw_posix_file_stop,
    .copy = ufr_gtw_posix_file_copy,
    .read = ufr_gtw_posix_file_read,
    .write = ufr_gtw_posix_file_write,
    .recv = ufr_gtw_posix_file_recv,
    .test_args = ufr_gtw_posix_file_test_args
};

// ============================================================================
//  Stdout Functions
// ============================================================================

static
int ufr_gtw_posix_stdout_boot(link_t* link, const ufr_args_t* args) {
    link->gtw_shr = NULL;
    link->gtw_obj = new_gtw_obj(stdout, 0);

    // success
	return UFR_OK;
}

static
int ufr_gtw_posix_stdout_start(link_t* link, int type, const ufr_args_t* args) {
    // success
	return UFR_OK;
}

static
void ufr_gtw_posix_stdout_stop(link_t* link, int type) {

}

static
ufr_gtw_api_t ufr_gtw_posix_stdout_api = {
    .name = "Posix:File",
    .type = ufr_gtw_posix_file_type,
    .state = ufr_gtw_posix_file_state,
    .size = ufr_gtw_posix_file_size,
    .boot = ufr_gtw_posix_stdout_boot,
    .start = ufr_gtw_posix_stdout_start,
    .stop = ufr_gtw_posix_stdout_stop,
    .copy = ufr_gtw_posix_file_copy,
    .read = ufr_gtw_posix_file_read,
    .write = ufr_gtw_posix_file_write,
    .recv = ufr_gtw_posix_file_recv
};

// ============================================================================
//  Stdout Functions
// ============================================================================

static
int ufr_gtw_posix_stdin_boot(link_t* link, const ufr_args_t* args) {
    link->gtw_shr = NULL;
    link->gtw_obj = new_gtw_obj(stdin, RECV_BUFFER_SIZE_DEFAULT);
	return UFR_OK;
}

static
int ufr_gtw_posix_stdin_start(link_t* link, int type, const ufr_args_t* args) {
    // success
	return UFR_OK;
}

static
void ufr_gtw_posix_stdin_stop(link_t* link, int type) {

}

static
ufr_gtw_api_t ufr_gtw_posix_stdin_api = {
    .name = "Posix:File",
    .type = ufr_gtw_posix_file_type,
    .state = ufr_gtw_posix_file_state,
    .size = ufr_gtw_posix_file_size,
    .boot = ufr_gtw_posix_stdin_boot,
    .start = ufr_gtw_posix_stdin_start,
    .stop = ufr_gtw_posix_stdin_stop,
    .copy = ufr_gtw_posix_file_copy,
    .read = ufr_gtw_posix_file_read,
    .write = ufr_gtw_posix_file_write,
    .recv = ufr_gtw_posix_file_recv
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_posix_new_file(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_posix_file_api;
    link->type_started = type;
    return UFR_OK;
}

int ufr_gtw_posix_new_stdout(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_posix_stdout_api;
    link->type_started = type;
    return UFR_OK;
}

int ufr_gtw_posix_new_stdin(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_posix_stdin_api;
    link->type_started = type;
    return UFR_OK;
}
