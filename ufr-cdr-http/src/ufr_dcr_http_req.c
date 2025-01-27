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
    const char *method;
    size_t method_len;
    const char *path;
    size_t path_len;
    const char *getvar;
    size_t getvar_len;
    int minor_version;
    struct phr_header headers[HEADER_MAX];
    size_t num_headers;

    // index of data
    int16_t idx;
} ll_decoder_t;

// ============================================================================
//  MsgPack Root
// ============================================================================

static
int ufr_dcr_http_req_boot(link_t* link, const ufr_args_t* args) {
    // create decoder object
    ll_decoder_t* dcr = malloc(sizeof(ll_decoder_t));
    dcr->method = NULL;
    dcr->method_len = 0;
    dcr->path = NULL;
    dcr->path_len = 0;
    dcr->num_headers = HEADER_MAX;
    dcr->idx = 0;

    // success
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_http_req_close(link_t* link) {

}

static
int ufr_dcr_http_req_next(link_t* link) {
    ll_decoder_t* dcr = link->dcr_obj;
    dcr->idx += 1;
	return UFR_OK;
}

static
int ufr_dcr_http_req_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = link->dcr_obj;

    if ( msg_size == 0 ) {
        return -1;
    }

    // initialize the dcr
    dcr->method = "";
    dcr->method_len = 0;
    dcr->path = "";
    dcr->path_len = 0;
    dcr->minor_version = 0;

    // num_header is updated by phr_parse_request, so started with default value
    dcr->num_headers = HEADER_MAX;

    const int ret = phr_parse_request (
        msg_data, msg_size,
        &dcr->method, &dcr->method_len,
        &dcr->path, &dcr->path_len,
        &dcr->minor_version,
        dcr->headers, &dcr->num_headers,
        0);

    // -2 if request is partial, -1 if failed

    if (ret > 0) {
        ufr_log(link, "received %d", ret);

        // Split Path to path?getvar; Example teste.com?id=aaa -> [teste.com, id=aaa]
        dcr->getvar = NULL;
        dcr->getvar_len = 0;
        for ( size_t i=0; i<dcr->path_len; i++ ) {
            if ( dcr->path[i] == '?' ) {
                dcr->path_len = i;
                dcr->getvar = &dcr->path[i+1];
                dcr->getvar_len = dcr->path_len - (i+1);
            }
        }

        // Success
        dcr->idx = 0;
        return UFR_OK;
    } else {
        // printf("ERROR(%ld) %.*s\n", msg_size , msg_size, msg_data);
    }

    // Error
    return ret;
}

static
char ufr_dcr_http_req_get_type(link_t* link) {
    ll_decoder_t* dcr = link->dcr_obj;

	return 0;
}

static
int ufr_dcr_http_req_get_nbytes(link_t* link) {
    int nbytes = 0;
    ll_decoder_t* dcr = link->dcr_obj;

    if ( dcr->idx == 0 ) {
        nbytes = dcr->method_len;
    } else if ( dcr->idx == 1 ) {
        nbytes = dcr->path_len;
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
int ufr_dcr_http_req_get_nitems(link_t* link) {

}

static
uint8_t* ufr_dcr_http_req_get_raw_ptr(link_t* link) {
    return NULL;
}

static
int ufr_dcr_http_req_get_raw(link_t* link, uint8_t* out_val, int maxlen) {
    ll_decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_req_get_u32(link_t* link, uint32_t out_val[], int max_nitems) {
    ll_decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_req_get_i32(link_t* link, int32_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_req_get_f32(link_t* link, float out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_req_get_u64(link_t* link, uint64_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_req_get_i64(link_t* link, int64_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_req_get_f64(link_t* link, double out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }
    int wrote = 0;


    return wrote;
}

static
int ufr_dcr_http_req_get_str(link_t* link, char* out_val, int maxlen) {
    ll_decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL ) {
        return 0;
    }

    int wrote = 0;

    if ( dcr->idx == 0 ) {
        strncpy(out_val, dcr->method, dcr->method_len);
        wrote = dcr->method_len;
    } else if ( dcr->idx == 1 ) {
        strncpy(out_val, dcr->path, dcr->path_len);
        wrote = dcr->path_len;
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
    ufr_dcr_http_req_next(link);
    return wrote;
}

int ufr_dcr_http_req_enter(link_t* link) {
    return -1;
}

int ufr_dcr_http_req_leave(link_t* link) {
    return -1;
}

static
ufr_dcr_api_t ufr_dcr_http_req_api = {
    .boot = ufr_dcr_http_req_boot,
    .close = ufr_dcr_http_req_close,

    .recv_cb = ufr_dcr_http_req_recv_cb,
    .recv_async_cb = ufr_dcr_http_req_recv_cb,
    .next = ufr_dcr_http_req_next,

    // metadata
    .get_type = ufr_dcr_http_req_get_type,
    .get_nbytes = ufr_dcr_http_req_get_nbytes,
    .get_nitems = ufr_dcr_http_req_get_nitems,
    .get_raw_ptr = ufr_dcr_http_req_get_raw_ptr,

    // 32 bits
    .get_u32 = ufr_dcr_http_req_get_u32,
    .get_i32 = ufr_dcr_http_req_get_i32,
    .get_f32 = ufr_dcr_http_req_get_f32,

    // 64 bits
    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

    // 8 bits
    .get_raw = ufr_dcr_http_req_get_raw,
    .get_str = ufr_dcr_http_req_get_str,

    // enter/leave
    .enter = ufr_dcr_http_req_enter,
    .leave = ufr_dcr_http_req_leave
};

// ============================================================================
//  Public
// ============================================================================

int ufr_dcr_http_ans_new(link_t* link, int type);

int ufr_dcr_http_req_new(link_t* link, int type) {
    link->dcr_api = &ufr_dcr_http_req_api;
    return UFR_OK;
}

int ufr_dcr_http_new(link_t* link, int type) {
    if ( type == UFR_START_CLIENT ) {
        ufr_dcr_http_ans_new(link, type);
    } else if ( type == UFR_START_SERVER ) {
        ufr_dcr_http_req_new(link, type);
    }
}