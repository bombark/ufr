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
	msgpack_unpacked result;
	char* msg_data;
	uint32_t msg_size;
    size_t cursor;
} ll_decoder_t;

// ============================================================================
//  private functions
// ============================================================================

static
msgpack_object unpack_next_obj(link_t* link) {
	ll_decoder_t* decoder = link->dcr_obj;
	size_t current = decoder->cursor;
	msgpack_unpack_return ret = msgpack_unpack_next(&decoder->result, decoder->msg_data, decoder->msg_size, &current);
	if ( ret != MSGPACK_UNPACK_SUCCESS ) {
		msgpack_object nulo = {.type=MSGPACK_OBJECT_NIL, .via.u64=0};
		return nulo;
	}
	decoder->cursor = current;
	return decoder->result.data;
}


// ============================================================================
//  msgpack
// ============================================================================

static
int ufr_dcr_msgpack_boot(link_t* link, const ufr_args_t* args) {
	ll_decoder_t* dcr_obj = malloc( sizeof(ll_decoder_t) );
	if ( dcr_obj == NULL ) {
		return ufr_error(link, ENOMEM, strerror(ENOMEM));
	}
	msgpack_unpacked_init(&dcr_obj->result);
	link->dcr_obj = dcr_obj;
	return UFR_OK;
}

static
void ufr_dcr_msgpack_close(link_t* link) {
	if ( link->dcr_obj != NULL ) {
		free(link->dcr_obj);
		link->dcr_obj = NULL;
	}
}

static
void ufr_dcr_msgpack_recv(link_t* link, char* msg_data, size_t msg_size) {
	ll_decoder_t* decoder = link->dcr_obj;
	decoder->msg_data = msg_data;
	decoder->msg_size = msg_size;
    decoder->cursor = 0;
}

static
int ufr_dcr_msgpack_get_i32(link_t* link, int32_t* val) {
	ll_decoder_t* load = link->dcr_obj;
	if ( load == NULL ) {
		return false;
	}

	// return the value
	const msgpack_object obj = unpack_next_obj(link); 
	if ( obj.type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
		*val = (int) obj.via.i64;
    } else if ( obj.type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        *val = (int) obj.via.i64;
    } else if ( obj.type == MSGPACK_OBJECT_FLOAT32 ) {
        *val = (int) obj.via.f64;
	} else {
		return false;
	}

	// success
	return true;
}

static
int ufr_dcr_msgpack_get_f32(link_t* link, float* val) {
	ll_decoder_t* load = link->dcr_obj;
	if ( load == NULL ) {
		return false;
	}

	// return the value
	const msgpack_object obj = unpack_next_obj(link); 
	if ( obj.type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
		*val = (float) obj.via.f64;
    } else if ( obj.type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        *val = (float) obj.via.f64;
    } else if ( obj.type == MSGPACK_OBJECT_FLOAT32 ) {
        *val = (float) obj.via.f64;
	} else {
		return false;
	}

	// success
	return true;
}

static
int ufr_dcr_msgpack_get_str(link_t* link, char** str) {
	*str = NULL;
	ll_decoder_t* load = link->dcr_obj;
	if ( load == NULL ) {
		return 1;
	}

	// link->idx[0].u64 = read_buffer(load, link->idx[0].u64);
	const msgpack_object obj = unpack_next_obj(link);
	if ( obj.type == MSGPACK_OBJECT_STR ) {
		*str = (char*) obj.via.str.ptr;
		return UFR_OK;
	}

	return 1;
}

static
int ufr_dcr_msgpack_copy_str(link_t* link, char* buffer, size_t size_max) {
    buffer[0] = '\0';
	ll_decoder_t* load = link->dcr_obj;
	if ( load == NULL ) {
		return 1;
	}

	// link->idx[0].u64 = read_buffer(load, link->idx[0].u64);
	const msgpack_object obj = unpack_next_obj(link); 
	if ( obj.type == MSGPACK_OBJECT_STR ) {
		strncpy(buffer, obj.via.str.ptr, obj.via.str.size);
        buffer[obj.via.str.size] = '\0';
		return UFR_OK;
	} else if ( obj.type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
		const size_t copied = snprintf(buffer, size_max, "%lu", obj.via.u64);
        buffer[copied] = '\0';
        return UFR_OK;
    } else if ( obj.type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        const size_t copied = snprintf(buffer, size_max, "%ld", obj.via.i64);
        buffer[copied] = '\0';
        return UFR_OK;
    } else if ( obj.type == MSGPACK_OBJECT_FLOAT32 ) {
        const size_t copied = snprintf(buffer, size_max, "%g", obj.via.f64);
        buffer[copied] = '\0';
        return UFR_OK;
    } else if ( obj.type == MSGPACK_OBJECT_FLOAT64 ) {
        const size_t copied = snprintf(buffer, size_max, "%g", obj.via.f64);
        buffer[copied] = '\0';
        return UFR_OK;
    }

	return 1;
}

static
int ufr_dcr_msgpack_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	ll_decoder_t* load = link->dcr_obj;
	if ( load == NULL ) {
		return 1;
	}

	const msgpack_object obj = unpack_next_obj(link); 
	if ( obj.type == MSGPACK_OBJECT_ARRAY ) {
		// *str = obj.via.str.ptr;
		return UFR_OK;
	}

	return 1;
}

static
int ufr_dcr_msgpack_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	ll_decoder_t* load = link->dcr_obj;
	if ( load == NULL ) {
		return ufr_error(link, 1, "decoder object is null");
	}

	const msgpack_object obj = unpack_next_obj(link); 
	if ( obj.type != MSGPACK_OBJECT_ARRAY ) {
		return ufr_error(link, 1, "item is not a array");
	}

	const size_t l_arr_size = 
		(obj.via.array.size > arr_size_max) ? arr_size_max : obj.via.array.size;

	if ( arr_type == 'i' ) {
		int32_t* out = (int32_t*) arr_ptr;
		for (size_t i=0; i<l_arr_size; i++) {
			out[i] = obj.via.array.ptr[i].via.i64;
		}

	} else if ( arr_type == 'f' ) {
		float* out = (float*) arr_ptr;
		for (size_t i=0; i<l_arr_size; i++) {
			out[i] = obj.via.array.ptr[i].via.f64;
		}
	}

	*arr_size = l_arr_size;
	return UFR_OK;
}

static
ufr_dcr_api_t ufr_dcr_msgpack_api = {
	.boot = ufr_dcr_msgpack_boot,
	.close = ufr_dcr_msgpack_close,

	.recv = ufr_dcr_msgpack_recv,

	.get_u32 = NULL,
	.get_i32 = ufr_dcr_msgpack_get_i32,
	.get_f32 = ufr_dcr_msgpack_get_f32,
	.get_str = ufr_dcr_msgpack_get_str,
    .copy_str = ufr_dcr_msgpack_copy_str,
	.get_arr = ufr_dcr_msgpack_get_arr,
	.copy_arr = ufr_dcr_msgpack_copy_arr
};

// ============================================================================
//  Public
// ============================================================================

int ufr_dcr_msgpack_new_obj(link_t* link, const int type) {
	link->dcr_api = &ufr_dcr_msgpack_api;
	return UFR_OK;
}
