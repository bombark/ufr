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

#define TOKEN_SIZE 1024

typedef struct {
    char const* msg_ptr;
    size_t msg_size;
    size_t msg_index;
    char sep; 
} ll_decoder_t;

// ============================================================================
//  Private Functions
// ============================================================================

void lex_next_token(ll_decoder_t* decoder, char* out_token) {
    const char sep = decoder->sep;
    const char* msg = decoder->msg_ptr;
    const size_t size = decoder->msg_size;
    size_t cursor = decoder->msg_index;
    size_t i_token = 0;

    while( cursor < size ) {
        const char c = msg[cursor];
        if ( c == sep ) {
            cursor += 1;
            break;
        } else if ( c == '\n' || c == '\0' ) {
            break;
        } else {
            if ( i_token < TOKEN_SIZE-1 ) {
                out_token[i_token] = c;
                i_token += 1;
            }
        }

        cursor += 1;
    }

    out_token[i_token] = '\0';
    decoder->msg_index = cursor;
}

// ============================================================================
//  CSV Decoder
// ============================================================================

static
int ufr_dcr_csv_boot(link_t* link, const ufr_args_t* args) {
    // allocate the decoder object
    ll_decoder_t* decoder = malloc(sizeof(ll_decoder_t));
    if ( decoder == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    // prepare the decoder
    const char* sep = ufr_args_gets(args, "@sep", ",");
    decoder->sep = sep[0];
    link->dcr_obj = (void*) decoder;
    return UFR_OK;
}

static
void ufr_dcr_csv_close(link_t* link) {
    if ( link->dcr_obj != NULL ) {
        free(link->dcr_obj);
        link->dcr_obj = NULL;
    }
}

static
void ufr_dcr_csv_recv(link_t* link, char* msg_data, size_t msg_size) {
	ll_decoder_t* decoder = link->dcr_obj;

    // initialize the decoder object with the new line
    decoder->msg_ptr = msg_data;
    decoder->msg_size = msg_size;
    decoder->msg_index = 0;

    // ufr_dbg("%s", msg_data);
}

static
int ufr_dcr_csv_get_i32(link_t* link, int32_t* val) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    lex_next_token(decoder, token);
    *val = atoi(token);
    return 0;
}

static
int ufr_dcr_csv_get_f32(link_t* link, float* val) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    lex_next_token(decoder, token);
    *val = atof(token);
    return 0;
}

static
int ufr_dcr_csv_get_str(link_t* link, char** str) {
	*str = NULL;
	ll_decoder_t* decoder = link->dcr_obj;
	return 0;
}

static
int ufr_dcr_csv_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	ll_decoder_t* decoder = link->dcr_obj;
	
}

static
int ufr_dcr_csv_copy_str(link_t* link, char* ret_val, size_t size_max) {
    ll_decoder_t* decoder = link->dcr_obj;
    lex_next_token(decoder, ret_val);
}

static
int ufr_dcr_csv_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	ll_decoder_t* decoder = link->dcr_obj;
	
	return UFR_OK;
}

static
ufr_dcr_api_t ufr_dcr_std_csv_api = {
    .boot = ufr_dcr_csv_boot,
    .close = ufr_dcr_csv_close,

	.recv = ufr_dcr_csv_recv,

	.get_u32 = NULL,
	.get_i32 = ufr_dcr_csv_get_i32,
	.get_f32 = ufr_dcr_csv_get_f32,
	.get_str = ufr_dcr_csv_get_str,

	.get_arr = ufr_dcr_csv_get_arr,
    .copy_str = ufr_dcr_csv_copy_str,
	.copy_arr = ufr_dcr_csv_copy_arr
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_dcr_std_new_csv(link_t* link, int type) {
    link->dcr_api = &ufr_dcr_std_csv_api;
	return UFR_OK;
}
