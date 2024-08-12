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
	uint8_t* msg_data;
    uint8_t stack;
	uint32_t msg_size;
    size_t cursor;
    msgpack_object object;
} ll_decoder_t;

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
int ufr_dcr_msgpack_next(link_t* link) {
    // parse the next object in the message
	ll_decoder_t* decoder = link->dcr_obj;
	size_t current = decoder->cursor;
	msgpack_unpack_return ret = msgpack_unpack_next(&decoder->result, decoder->msg_data, decoder->msg_size, &current);

    // error
	if ( ret != MSGPACK_UNPACK_SUCCESS ) {
		decoder->object.type = MSGPACK_OBJECT_NIL;
        decoder->object.via.u64 = 0;
		return -1;
	}

    // update decoder object
	decoder->cursor = current;
    decoder->object = decoder->result.data;
	return UFR_OK;
}

static
void ufr_dcr_msgpack_recv(link_t* link, char* msg_data, size_t msg_size) {
	ll_decoder_t* decoder = link->dcr_obj;
	decoder->msg_data = msg_data;
	decoder->msg_size = msg_size;
    decoder->cursor = 0;
    ufr_dcr_msgpack_next(link);
}

static
char ufr_dcr_msgpack_get_type(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        return 'i';
    }

    if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        return 'f';
    }

    if ( type == MSGPACK_OBJECT_ARRAY ) {
        return 'a';
    }

    if ( type == MSGPACK_OBJECT_STR ) {
        return 's';
    }

    if ( type == MSGPACK_OBJECT_BIN ) {
        return 'r';
    }

	return 0;
}

static
size_t ufr_dcr_msgpack_get_size(link_t* link) {
	ll_decoder_t* decoder = link->dcr_obj;
    const int type = decoder->object.type;
    switch (type) {
        case MSGPACK_OBJECT_POSITIVE_INTEGER:
            return sizeof(uint64_t);

        case MSGPACK_OBJECT_NEGATIVE_INTEGER:
            return sizeof(int64_t);

        case MSGPACK_OBJECT_FLOAT32:
            return sizeof(float);

        case MSGPACK_OBJECT_FLOAT64:
            return sizeof(double);
        
        case MSGPACK_OBJECT_ARRAY:
            return decoder->object.via.array.size;

        case MSGPACK_OBJECT_STR:
            return decoder->object.via.str.size;

        case MSGPACK_OBJECT_BIN:
            return decoder->object.via.bin.size;

        default:
            return 0;
    }
}

static
uint8_t* ufr_dcr_msgpack_get_raw_ptr(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder->object.type != MSGPACK_OBJECT_BIN ) {
        return NULL;
    }
    return (uint8_t*) decoder->object.via.bin.ptr;
}


static
int ufr_dcr_msgpack_get_i32(link_t* link, int32_t* val) {
	ll_decoder_t* decoder = link->dcr_obj;
	if ( decoder == NULL ) {
		return -1;
	}

	// return the value
	const int type = decoder->object.type;
	if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
		*val = (int) decoder->object.via.u64;
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        *val = (int) decoder->object.via.i64;
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        *val = (int) decoder->object.via.f64;
	} else {
		return -1;
	}

	// success
    ufr_dcr_msgpack_next(link);
	return UFR_OK;
}

static
int ufr_dcr_msgpack_get_f32(link_t* link, float* val) {
	ll_decoder_t* decoder = link->dcr_obj;
	if ( decoder == NULL ) {
		return -1;
	}

	// return the value
    const int type = decoder->object.type;
	if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
		*val = (float) decoder->object.via.u64;
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        *val = (float) decoder->object.via.i64;
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        *val = (float) decoder->object.via.f64;
	} else {
		return -1;
	}

	// success
    ufr_dcr_msgpack_next(link);
	return UFR_OK;
}

static
int ufr_dcr_msgpack_get_str(link_t* link, char** str) {
	*str = NULL;
	ll_decoder_t* decoder = link->dcr_obj;
	if ( decoder == NULL ) {
		return 1;
	}

	// link->idx[0].u64 = read_buffer(load, link->idx[0].u64);
	const int type = decoder->object.type;
	if ( type == MSGPACK_OBJECT_STR ) {
		*str = (char*) decoder->object.via.str.ptr;
		return UFR_OK;
	}

    ufr_dcr_msgpack_next(link);
	return 1;
}

static
int ufr_dcr_msgpack_copy_str(link_t* link, char* buffer, size_t size_max) {
    buffer[0] = '\0';
	ll_decoder_t* decoder = link->dcr_obj;
	if ( decoder == NULL ) {
		return 1;
	}

	const int type = decoder->object.type;
	if ( type == MSGPACK_OBJECT_STR ) {
        const char* ptr = decoder->object.via.str.ptr;
        const size_t size = decoder->object.via.str.size;
		strncpy(buffer, ptr, size);
        buffer[size] = '\0';
	} else if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
		const size_t copied = snprintf(buffer, size_max, "%lu", decoder->object.via.u64);
        buffer[copied] = '\0';
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        const size_t copied = snprintf(buffer, size_max, "%ld", decoder->object.via.i64);
        buffer[copied] = '\0';
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        const size_t copied = snprintf(buffer, size_max, "%g", decoder->object.via.f64);
        buffer[copied] = '\0';
    } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        const size_t copied = snprintf(buffer, size_max, "%g", decoder->object.via.f64);
        buffer[copied] = '\0';
    } else {
        return 1;
    }

    // Success
    ufr_dcr_msgpack_next(link);
	return UFR_OK;
}

static
int ufr_dcr_msgpack_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	ll_decoder_t* load = link->dcr_obj;
	if ( load == NULL ) {
		return 1;
	}
/*
	const msgpack_object obj = unpack_next_obj(link); 
	if ( obj.type == MSGPACK_OBJECT_ARRAY ) {
		// *str = obj.via.str.ptr;
		return UFR_OK;
	}
*/

	return 1;
}

static
int ufr_dcr_msgpack_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	ll_decoder_t* decoder = link->dcr_obj;
	if ( decoder == NULL ) {
		return ufr_error(link, 1, "decoder object is null");
	}

    const int type = decoder->object.type;
    if ( arr_type == 'b' && type == MSGPACK_OBJECT_BIN ) {
        const size_t object_size = decoder->object.via.bin.size;
        const size_t l_arr_size = (object_size > arr_size_max) ? arr_size_max : object_size;
        memcpy(arr_ptr, decoder->object.via.bin.ptr, l_arr_size);
        *arr_size = l_arr_size;

    } else {
        if ( type != MSGPACK_OBJECT_ARRAY ) {
            return ufr_error(link, 1, "item is not a array");
        }

        const size_t object_size = decoder->object.via.bin.size;
        const size_t l_arr_size = (object_size > arr_size_max) ? arr_size_max : object_size;
        const msgpack_object_array object_array = decoder->object.via.array;
        if ( arr_type == 'i' ) {
            int32_t* out = (int32_t*) arr_ptr;
            for (size_t i=0; i<l_arr_size; i++) {
                out[i] = object_array.ptr[i].via.i64;
            }

        } else if ( arr_type == 'f' ) {
            float* out = (float*) arr_ptr;
            for (size_t i=0; i<l_arr_size; i++) {
                out[i] = object_array.ptr[i].via.f64;
            }

        }

        *arr_size = l_arr_size;
    }

	return UFR_OK;
}

int ufr_dcr_msgpack_get_ai32(link_t* link, ufr_ai32_t* out_array) {
    ll_decoder_t* decoder = link->dcr_obj;
	if ( decoder == NULL ) {
		return ufr_error(link, 1, "decoder object is null");
	}

	const int type = decoder->object.type;
    if ( type != MSGPACK_OBJECT_ARRAY ) {
		return ufr_error(link, 1, "item is not a array");
	}

    // alloc memory for the copy
    const msgpack_object_array array = decoder->object.via.array;
    int32_t* data = malloc(sizeof(int32_t) * array.size);
    if ( data == NULL ) {
        return ufr_error(link, 1, "no memory");
    }

    // copy array
    for (size_t i=0; i<array.size; i++) {
        data[i] = array.ptr[i].via.i64;
    }

    // end
    out_array->size = array.size;
    out_array->data = data;
    ufr_dcr_msgpack_next(link);
    return UFR_OK;
}

static
ufr_dcr_api_t ufr_dcr_msgpack_api = {
	.boot = ufr_dcr_msgpack_boot,
	.close = ufr_dcr_msgpack_close,

	.recv = ufr_dcr_msgpack_recv,
    .next = ufr_dcr_msgpack_next,

    .get_type = ufr_dcr_msgpack_get_type,
    .get_size = ufr_dcr_msgpack_get_size,
    .get_raw_ptr = ufr_dcr_msgpack_get_raw_ptr,

	.get_u32 = NULL,
	.get_i32 = ufr_dcr_msgpack_get_i32,
	.get_f32 = ufr_dcr_msgpack_get_f32,
	.get_str = ufr_dcr_msgpack_get_str,
    .get_ai32 = ufr_dcr_msgpack_get_ai32,
    .copy_str = ufr_dcr_msgpack_copy_str,
	.get_arr = ufr_dcr_msgpack_get_arr,
	.copy_arr = ufr_dcr_msgpack_copy_arr
};

// ============================================================================
//  Public
// ============================================================================

int ufr_dcr_msgpack_new(link_t* link, const int type) {
	link->dcr_api = &ufr_dcr_msgpack_api;
	return UFR_OK;
}
