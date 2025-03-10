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
#include <msgpack.h>
#include <ufr.h>

typedef struct {
	msgpack_sbuffer sbuf;
	msgpack_packer pk;
} ll_encoder_t;

// ============================================================================
//  MsgPack Driver
// ============================================================================

static
int ufr_enc_msgpack_boot(link_t* link, const ufr_args_t* args) {
	ll_encoder_t* enc_obj = malloc( sizeof(ll_encoder_t) );
	if ( enc_obj == NULL ) {
		return ufr_error(link, ENOMEM, strerror(ENOMEM));
	}
	msgpack_sbuffer_init(&enc_obj->sbuf);
	msgpack_packer_init(&enc_obj->pk, &enc_obj->sbuf, msgpack_sbuffer_write);
	link->enc_obj = enc_obj;
	return UFR_OK;
}

static
void ufr_enc_msgpack_close(link_t* link) {
	if ( link->dcr_obj != NULL ) {
		free(link->dcr_obj);
		link->dcr_obj = NULL;
	}
}

static
int ufr_enc_msgpack_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_uint32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_int32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
		    msgpack_pack_float(&enc_obj->pk, val[wrote]);
        }
	}
	return wrote;
}

static
int ufr_enc_msgpack_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_uint64(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_int64(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
		    msgpack_pack_double(&enc_obj->pk, val[wrote]);
        }
	}
	return wrote;
}


static
int ufr_enc_msgpack_put_str(link_t* link, const char* val) {
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj ) {
		const size_t size = strlen(val);
		msgpack_pack_str(&enc_obj->pk, size);
		msgpack_pack_str_body(&enc_obj->pk, val, size);
	}
	return 0;
}

static
int ufr_enc_msgpack_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( type == 'i' ) {
		const int* ai32_ptr = (int*) arr_ptr;
		msgpack_pack_array(&enc_obj->pk, arr_size);
		for (size_t i=0; i<arr_size; i++) {
			msgpack_pack_int(&enc_obj->pk, ai32_ptr[i]);
		}
	} else if ( type == 'f' ) {
		const float* af32_ptr = (float*) arr_ptr;
		msgpack_pack_array(&enc_obj->pk, arr_size);
		for (size_t i=0; i<arr_size; i++) {
			msgpack_pack_float(&enc_obj->pk, af32_ptr[i]);
		}

	}
}

static
int ufr_enc_msgpack_put_cmd(link_t* link, char cmd) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( cmd == '\n' ) {
        const size_t size = enc_obj->sbuf.size;
        const char* data = enc_obj->sbuf.data;
        ufr_write(link, data, size);
        msgpack_sbuffer_clear(&enc_obj->sbuf);
    } else if ( cmd == (char) EOF ) {
        ufr_write(link, NULL, 0);
        msgpack_sbuffer_clear(&enc_obj->sbuf);
    } else {
        return ufr_error(link, 1, "Command %d not found", cmd);
    }

    return UFR_OK;
}

static
int ufr_enc_msgpack_put_raw(link_t* link, const uint8_t* buffer, int size) {
	ll_encoder_t* enc_obj = link->enc_obj;
	if ( enc_obj == NULL ) {
		return -1;
	}

	msgpack_pack_bin(&enc_obj->pk, size);
	msgpack_pack_bin_body(&enc_obj->pk, buffer, size);
	return size;
}

int ufr_enc_msgpack_enter(link_t* link, size_t maxsize) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    msgpack_pack_array(&enc_obj->pk, maxsize);
    return UFR_OK;
}


int ufr_enc_msgpack_leave(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_msgpack_api = {
	.boot = ufr_enc_msgpack_boot,
	.close = ufr_enc_msgpack_close,

	.put_u32 = ufr_enc_msgpack_put_u32,
	.put_i32 = ufr_enc_msgpack_put_i32,
	.put_f32 = ufr_enc_msgpack_put_f32,

	.put_u64 = ufr_enc_msgpack_put_u64,
	.put_i64 = ufr_enc_msgpack_put_i64,
	.put_f64 = ufr_enc_msgpack_put_f64,

	.put_cmd = ufr_enc_msgpack_put_cmd,
	.put_str = ufr_enc_msgpack_put_str,
    .put_raw = ufr_enc_msgpack_put_raw,
	
	.enter = ufr_enc_msgpack_enter,
	.leave = ufr_enc_msgpack_leave
};

// ============================================================================
//  Public
// ============================================================================

int ufr_enc_msgpack_new(link_t* link) {
	link->enc_api = &ufr_enc_msgpack_api;
	return UFR_OK;
}

