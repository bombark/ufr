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
    int code;
    char msg[64];
    char type[128];
    char* body_length_ptr;
    int header_length;

    ufr_buffer_t body;
    uint8_t idx;
    uint8_t idx_write;
} ll_encoder_t;

// ============================================================================
//  MsgPack Driver
// ============================================================================

static
int ufr_enc_http_ans_boot(link_t* link, const ufr_args_t* args) {
	ll_encoder_t* enc = malloc( sizeof(ll_encoder_t) );
	if ( enc == NULL ) {
		return ufr_error(link, ENOMEM, strerror(ENOMEM));
	}

    // initialize the encoder
    enc->code = 0;
    enc->msg[0] = '\0';
    enc->type[0] = '\0';
    enc->idx = 0;
    enc->idx_write = 0;
    ufr_buffer_init(&enc->body);

    // success
	link->enc_obj = enc;
	return UFR_OK;
}

static
void ufr_enc_http_ans_close(link_t* link) {
	if ( link->enc_obj != NULL ) {
		free(link->enc_obj);
		link->enc_obj = NULL;
	}
}

static
int ufr_enc_http_ans_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
    }
    return wrote;
}

static
int ufr_enc_http_ans_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc = link->enc_obj;
    if ( enc ) {
        for (; wrote<nitems; wrote++) {
            const int val_i32 = val[wrote];
            if ( enc->idx == 0 ) {
                enc->code = val_i32;
                if ( val_i32 == 200 ) {
                    strcpy(enc->msg, "OK");
                    enc->idx += 1;
                } else if ( val_i32 == 404 ) {
                    strcpy(enc->msg, "Not found");
                    enc->idx += 1;
                }

                char buffer[1024];
                int size1 = snprintf(buffer, 1024, "HTTP/1.1 %d %s\r\n", enc->code, enc->msg);
                ufr_buffer_put(&enc->body, buffer, size1);
            } else {
                break;
            }
        }
    }
    return wrote;
}

static
int ufr_enc_http_ans_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
	}
	return wrote;
}

static
int ufr_enc_http_ans_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
    }
    return wrote;
}

static
int ufr_enc_http_ans_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
    }
    return wrote;
}

static
int ufr_enc_http_ans_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {

        }
	}
	return wrote;
}


static
int ufr_enc_http_ans_put_str(link_t* link, const char* val) {
    
	ll_encoder_t* enc = link->enc_obj;
	if ( enc ) {
        char buffer[1024];

        if ( enc->idx == 0 ) {
            
        } else if ( enc->idx == 1 ) {
            int size2 = snprintf(buffer, 1024, "Content-Type: %s\r\n", val);
            ufr_buffer_put(&enc->body, buffer, size2);

            /* Connection: keep-alive
            int size4 = snprintf(buffer, 1024, "Connection: close\r\n", val);
            ufr_buffer_put(&enc->body, buffer, size4);
            */

            // save the location of content length
            int size3 = snprintf(buffer, 1024, "Content-Length: ");
            ufr_buffer_put(&enc->body, buffer, size3);
            enc->body_length_ptr = &enc->body.ptr[ enc->body.size ];

            // create space for content length
            ufr_buffer_put(&enc->body, "                \r\n\r\n", 20);

            // save the header length
            enc->header_length = enc->body.size;
        } else if ( enc->idx >= 2 ) {
            ufr_buffer_put(&enc->body, val, strlen(val));
        }
        enc->idx += 1;
	}
	return 0;
}


static
int ufr_enc_http_ans_put_cmd(link_t* link, char cmd) {
    ll_encoder_t* enc = link->enc_obj;
    if ( cmd == '\n' ) {

    } else if ( cmd == (char) EOF ) {
        char buffer[16];
        int body_size = enc->body.size - enc->header_length;
        int buffer_len = snprintf(buffer, 16, "%d", body_size);
        strncpy(enc->body_length_ptr, buffer, buffer_len);

        // ufr_write(link, buffer, size3);
        // printf("%.*s\n", enc->body.size, enc->body.ptr);

        ufr_write(link, enc->body.ptr, enc->body.size);
        ufr_write(link, NULL, 0);
        // printf("%s\n", enc->body.ptr);

        ufr_buffer_clear(&enc->body);
        enc->idx = 0;
    } else {
        return ufr_error(link, 1, "Command %d not found", cmd);
    }

    return UFR_OK;
}

static
int ufr_enc_http_ans_put_raw(link_t* link, const uint8_t* buffer, int size) {

	ll_encoder_t* enc = link->enc_obj;
	if ( enc == NULL ) {
		return -1;
	}

    if ( enc->idx >= 2 ) {
        ufr_buffer_put(&enc->body, buffer, size);
    } else {
        return 0;
    }

	return size;
}

int ufr_enc_http_ans_enter(link_t* link, size_t maxsize) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;

    return UFR_OK;
}


int ufr_enc_http_ans_leave(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_http_ans_api = {
	.boot = ufr_enc_http_ans_boot,
	.close = ufr_enc_http_ans_close,

	.put_u32 = ufr_enc_http_ans_put_u32,
	.put_i32 = ufr_enc_http_ans_put_i32,
	.put_f32 = ufr_enc_http_ans_put_f32,

	.put_u64 = ufr_enc_http_ans_put_u64,
	.put_i64 = ufr_enc_http_ans_put_i64,
	.put_f64 = ufr_enc_http_ans_put_f64,

	.put_cmd = ufr_enc_http_ans_put_cmd,
	.put_str = ufr_enc_http_ans_put_str,
    .put_raw = ufr_enc_http_ans_put_raw,
	
	.enter = ufr_enc_http_ans_enter,
	.leave = ufr_enc_http_ans_leave
};

// ============================================================================
//  Public
// ============================================================================

int ufr_enc_http_req_new(link_t* link, int type);

int ufr_enc_http_ans_new(link_t* link, int type) {
	link->enc_api = &ufr_enc_http_ans_api;
	return UFR_OK;
}

int ufr_enc_http_new(link_t* link, int type) {
    if ( type == UFR_START_CLIENT ) {
        ufr_enc_http_req_new(link, type);
    } else if ( type == UFR_START_SERVER ) {
        ufr_enc_http_ans_new(link, type);
    }
}