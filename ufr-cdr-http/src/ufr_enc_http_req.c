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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ufr.h>

typedef struct {
    char method[32];
    char path[4096];
    // ufr_buffer_t body;
    uint8_t idx;
} ll_encoder_t;

// ============================================================================
//  MsgPack Driver
// ============================================================================

static
int ufr_enc_http_req_boot(link_t* link, const ufr_args_t* args) {
	ll_encoder_t* enc = malloc( sizeof(ll_encoder_t) );
	if ( enc == NULL ) {
		return ufr_error(link, ENOMEM, strerror(ENOMEM));
	}

    // initialize the encoder
    enc->method[0] = '\0';
    enc->path[0] = '\0';
    enc->idx = 0;
    // ufr_buffer_init(&enc->body);

    // success
	link->enc_obj = enc;
	return UFR_OK;
}

static
void ufr_enc_http_req_close(link_t* link) {
	if ( link->enc_obj != NULL ) {
		free(link->enc_obj);
		link->enc_obj = NULL;
	}
}

static
int ufr_enc_http_req_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
    }
    return wrote;
}

static
int ufr_enc_http_req_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
    }
    return wrote;
}

static
int ufr_enc_http_req_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
	}
	return wrote;
}

static
int ufr_enc_http_req_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
    }
    return wrote;
}

static
int ufr_enc_http_req_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
    }
    return wrote;
}

static
int ufr_enc_http_req_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
	}
	return wrote;
}


static
int ufr_enc_http_req_put_str(link_t* link, const char* val) {
	ll_encoder_t* enc = link->enc_obj;
	if ( enc ) {
        if ( enc->idx == 0 ) {
            strcpy(enc->method, val);
        } else if ( enc->idx == 1 ) {
            strcpy(enc->path, val);
        }
        enc->idx += 1;
	}
	return 0;
}


static
int ufr_enc_http_req_put_cmd(link_t* link, char cmd) {
    ll_encoder_t* enc = link->enc_obj;
    if ( cmd == '\n' ) {

    } else if ( cmd == (char) EOF ) {
        char buffer[4096];
        const size_t size = snprintf(buffer, 4096, "%s %s HTTP/1.1\r\n\r\n", enc->method, enc->path);
        ufr_write(link, buffer, size);

    } else {
        return ufr_error(link, 1, "Command %d not found", cmd);
    }

    return UFR_OK;
}

static
int ufr_enc_http_req_put_raw(link_t* link, const uint8_t* buffer, int size) {
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj == NULL ) {
		return -1;
	}


	return size;
}

int ufr_enc_http_req_enter(link_t* link, size_t maxsize) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;

    return UFR_OK;
}


int ufr_enc_http_req_leave(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_http_req_api = {
	.boot = ufr_enc_http_req_boot,
	.close = ufr_enc_http_req_close,

	.put_u32 = ufr_enc_http_req_put_u32,
	.put_i32 = ufr_enc_http_req_put_i32,
	.put_f32 = ufr_enc_http_req_put_f32,

	.put_u64 = ufr_enc_http_req_put_u64,
	.put_i64 = ufr_enc_http_req_put_i64,
	.put_f64 = ufr_enc_http_req_put_f64,

	.put_cmd = ufr_enc_http_req_put_cmd,
	.put_str = ufr_enc_http_req_put_str,
    .put_raw = ufr_enc_http_req_put_raw,
	
	.enter = ufr_enc_http_req_enter,
	.leave = ufr_enc_http_req_leave
};

// ============================================================================
//  Public
// ============================================================================

int ufr_enc_http_req_new(link_t* link, int type) {
	link->enc_api = &ufr_enc_http_req_api;
	return UFR_OK;
}