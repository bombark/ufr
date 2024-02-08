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
int ufr_ecr_msgpack_boot(link_t* link, const lt_args_t* args) {
	ll_encoder_t* ecr_obj = malloc( sizeof(ll_encoder_t) );
	if ( ecr_obj == NULL ) {
		return lt_error(link, ENOMEM, strerror(ENOMEM));
	}
	msgpack_sbuffer_init(&ecr_obj->sbuf);
	msgpack_packer_init(&ecr_obj->pk, &ecr_obj->sbuf, msgpack_sbuffer_write);
	link->ecr_obj = ecr_obj;
	return LT_OK;
}

static
void ufr_ecr_msgpack_close(link_t* link) {
	if ( link->dcr_obj != NULL ) {
		free(link->dcr_obj);
		link->dcr_obj = NULL;
	}
}

static
int ufr_ecr_msgpack_put_u32(link_t* link, uint32_t val) {
	ll_encoder_t* ecr_obj = link->ecr_obj;
	if ( ecr_obj ) {
		msgpack_pack_int32(&ecr_obj->pk, val);
	}
	return 0;
}

static
int ufr_ecr_msgpack_put_i32(link_t* link, int32_t val) {
	ll_encoder_t* ecr_obj = link->ecr_obj;
	if ( ecr_obj ) {
		msgpack_pack_int32(&ecr_obj->pk, val);
	}
	return 0;
}

static
int ufr_ecr_msgpack_put_f32(link_t* link, float val) {
	ll_encoder_t* ecr_obj = link->ecr_obj;
	if ( ecr_obj ) {
		msgpack_pack_float(&ecr_obj->pk, val);
	}
	return 0;
}

static
int ufr_ecr_msgpack_put_str(link_t* link, const char* val) {
	ll_encoder_t* ecr_obj = link->ecr_obj;
	if ( ecr_obj ) {
		const size_t size = strlen(val);
		msgpack_pack_str(&ecr_obj->pk, size);
		msgpack_pack_str_body(&ecr_obj->pk, val, size);
	}
	return 0;
}

static
int ufr_ecr_msgpack_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
	ll_encoder_t* ecr_obj = link->ecr_obj;
	if ( type == 'i' ) {
		const int* arr_i32_ptr = (int*) arr_ptr;
		msgpack_pack_array(&ecr_obj->pk, arr_size);
		for (size_t i=0; i<arr_size; i++) {
			msgpack_pack_int(&ecr_obj->pk, arr_i32_ptr[i]);
		}
	} else if ( type == 'f' ) {
		const float* arr_f32_ptr = (float*) arr_ptr;
		msgpack_pack_array(&ecr_obj->pk, arr_size);
		for (size_t i=0; i<arr_size; i++) {
			msgpack_pack_float(&ecr_obj->pk, arr_f32_ptr[i]);
		}

	}
}

static
int ufr_ecr_msgpack_put_cmd(link_t* link, char cmd) {
	ll_encoder_t* ecr_obj = link->ecr_obj;

	if ( cmd == '\n' ) {
		const size_t size = ecr_obj->sbuf.size;
		const char* data = ecr_obj->sbuf.data;
		lt_write(link, data, size);
		msgpack_sbuffer_clear(&ecr_obj->sbuf);
	}

	return 0;
}

int ufr_ecr_msgpack_enter_array(link_t* link, size_t maxsize) {
    ll_encoder_t* ecr_obj = (ll_encoder_t*) link->ecr_obj;
	msgpack_pack_array(&ecr_obj->pk, maxsize);
    return LT_OK;
}


int ufr_ecr_msgpack_leave_array(link_t* link) {
    ll_encoder_t* ecr_obj = (ll_encoder_t*) link->ecr_obj;
    return LT_OK;
}

static
lt_encoder_api_t ufr_ecr_msgpack_api = {
	.boot = ufr_ecr_msgpack_boot,
	.close = ufr_ecr_msgpack_close,

	.put_u32 = ufr_ecr_msgpack_put_u32,
	.put_i32 = ufr_ecr_msgpack_put_i32,
	.put_f32 = ufr_ecr_msgpack_put_f32,
	.put_str = ufr_ecr_msgpack_put_str,
	.put_arr = ufr_ecr_msgpack_put_arr,
	.put_cmd = ufr_ecr_msgpack_put_cmd,
	
	.enter_array = ufr_ecr_msgpack_enter_array,
	.leave_array = ufr_ecr_msgpack_leave_array
};

// ============================================================================
//  Public
// ============================================================================

int ufr_ecr_msgpack_new_obj(link_t* link, const int type) {
	link->ecr_api = &ufr_ecr_msgpack_api;
	return LT_OK;
}

