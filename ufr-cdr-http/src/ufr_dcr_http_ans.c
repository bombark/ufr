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

#include "picohttpparser.h"

#define HEADER_MAX 20

typedef struct {
    // data for picohttpparser
    const char *msg;
    size_t msg_len;
    int minor_version, status;
    struct phr_header headers[HEADER_MAX];
    size_t num_headers;
    const char* body;
    size_t body_len;
    const char* body_type;
    size_t body_type_len;

    // index of data
    int16_t idx;
} ll_decoder_t;

// ============================================================================
//  MsgPack Root
// ============================================================================

static
int ufr_dcr_http_ans_boot(link_t* link, const ufr_args_t* args) {
    // create decoder object
    ll_decoder_t* dcr = malloc(sizeof(ll_decoder_t));
    dcr->msg = NULL;
    dcr->msg_len = 0;
    dcr->minor_version = 0;
    dcr->status = 0;
    dcr->num_headers = HEADER_MAX;
    dcr->idx = 0;

    // success
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_http_ans_close(link_t* link) {

}

static
int ufr_dcr_http_ans_next(link_t* link) {
    ll_decoder_t* dcr = link->dcr_obj;
    dcr->idx += 1;
	return UFR_OK;
}

static
int ufr_dcr_http_ans_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = link->dcr_obj;

    const int ret = phr_parse_response (
        msg_data, msg_size,
        &dcr->minor_version, &dcr->status,
        &dcr->msg, &dcr->msg_len,
        dcr->headers, &dcr->num_headers,
        0);

    if (ret > 0) {
        dcr->body_type = "text/plain";
        dcr->body_type_len = 10;
        dcr->body_len = 0;
        dcr->body = &msg_data[ret];

        for ( int i=0; i<dcr->num_headers; i++ ) {
            const struct phr_header* header = &dcr->headers[i];
            if ( strncmp(header->name, "Content-Type", header->name_len) == 0 ) {
                dcr->body_type = header->name;
                dcr->body_type_len = header->name_len;
            } else if ( strncmp(header->name, "Content-Length", header->name_len) == 0 ) {
                dcr->body_len = atoi(header->value);
            }
        }

        dcr->idx = 0;
        return UFR_OK;
    }

    return ret;
}

static
char ufr_dcr_http_ans_get_type(link_t* link) {
    ll_decoder_t* dcr = link->dcr_obj;

	return 0;
}

static
int ufr_dcr_http_ans_get_nbytes(link_t* link) {
    int nbytes = 0;
    ll_decoder_t* dcr = link->dcr_obj;

    if ( dcr->idx == 0 ) {
        nbytes = sizeof(dcr->status);
    } else if ( dcr->idx == 1 ) {
        nbytes = dcr->msg_len;
    } else if ( dcr->idx < 0 ) {
        // nothing
    } else {
        const size_t header_idx = (dcr->idx - 2) / 2;
        if ( header_idx < dcr->num_headers ) {
            const struct phr_header* header = &dcr->headers[header_idx]; 
            if ( (dcr->idx % 2) == 0 ) {
                nbytes = header->name_len;
            } else {
                nbytes = header->value_len;
            }
        }
    }
    return nbytes;
}

static
int ufr_dcr_http_ans_get_nitems(link_t* link) {

}

static
uint8_t* ufr_dcr_http_ans_get_raw_ptr(link_t* link) {
    return NULL;
}

static
int ufr_dcr_http_ans_get_raw(link_t* link, uint8_t* out_val, int maxlen) {
    ll_decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_ans_get_u32(link_t* link, uint32_t out_val[], int max_nitems) {
    ll_decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_ans_get_i32(link_t* link, int32_t out_val[], int max_nitems) {
    ll_decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL ) {
        return 0;
    }
    int wrote = 0;

    if ( dcr->idx == 0 ) {
        out_val[0] = dcr->status;
    } 


    return wrote;
}

static
int ufr_dcr_http_ans_get_f32(link_t* link, float out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_ans_get_u64(link_t* link, uint64_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_ans_get_i64(link_t* link, int64_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_ans_get_f64(link_t* link, double out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_ans_get_str(link_t* link, char* out_val, int maxlen) {
    ll_decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL ) {
        return 0;
    }

    int wrote = 0;

    if ( dcr->idx == 0 ) {
        wrote = snprintf(out_val, maxlen, "%d", dcr->status);
    } else if ( dcr->idx == 1 ) {
        strncpy(out_val, dcr->body, dcr->body_len);
        wrote = dcr->body_len;
    } else if ( dcr->idx < 0 ) {
        // nothing
    } else {
        const size_t header_idx = (dcr->idx - 2) / 2;
        if ( header_idx < dcr->num_headers ) {
            const struct phr_header* header = &dcr->headers[header_idx]; 
            if ( (dcr->idx % 2) == 0 ) {
                strncpy(out_val, header->name, header->name_len );
                wrote = header->name_len;
            } else {
                strncpy(out_val, header->value, header->value_len );
                wrote = header->value_len;
            }
        }
    }

    // success
    out_val[wrote] = '\0';
    ufr_dcr_http_ans_next(link);
    return wrote;
}

int ufr_dcr_http_ans_enter(link_t* link) {
    return -1;
}

int ufr_dcr_http_ans_leave(link_t* link) {
    return -1;
}

static
ufr_dcr_api_t ufr_dcr_http_ans_api = {
    .boot = ufr_dcr_http_ans_boot,
    .close = ufr_dcr_http_ans_close,

    .recv_cb = ufr_dcr_http_ans_recv_cb,
    .recv_async_cb = ufr_dcr_http_ans_recv_cb,
    .next = ufr_dcr_http_ans_next,

    // metadata
    .get_type = ufr_dcr_http_ans_get_type,
    .get_nbytes = ufr_dcr_http_ans_get_nbytes,
    .get_nitems = ufr_dcr_http_ans_get_nitems,
    .get_raw_ptr = ufr_dcr_http_ans_get_raw_ptr,

    // 32 bits
    .get_u32 = ufr_dcr_http_ans_get_u32,
    .get_i32 = ufr_dcr_http_ans_get_i32,
    .get_f32 = ufr_dcr_http_ans_get_f32,

    // 64 bits
    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

    // 8 bits
    .get_raw = ufr_dcr_http_ans_get_raw,
    .get_str = ufr_dcr_http_ans_get_str,

    // enter/leave
    .enter = ufr_dcr_http_ans_enter,
    .leave = ufr_dcr_http_ans_leave
};

// ============================================================================
//  Public
// ============================================================================

int ufr_dcr_http_ans_new(link_t* link, int type) {
    link->dcr_api = &ufr_dcr_http_ans_api;
    return UFR_OK;
}
