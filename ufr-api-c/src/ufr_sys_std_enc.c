/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ufr.h"

// ============================================================================
//  Functions
// ============================================================================

bool str_needs_quote(const char* str, const size_t size) {
    bool is_need = false;

    // check if the first is '[' or ']'
    const char first = str[0];
    if ( first == '[' || first == ']' ) {
        return true;
    }

    // Check if string has ' '
    for (size_t i=0; i<size; i++) {
        const char c = str[i];
        if ( c == ' ' ) {
            is_need = true;
            break;
        }
    }

    // end
    return is_need;
}

void copy_str_replacing_symbols(ufr_buffer_t* buffer, const char* str, const size_t size) {
    for (size_t i=0; i<size; i++) {
        const char c = str[i];
        if ( c == '\n' ) {
            ufr_buffer_put_str(buffer, "\\n");
        } else {
            ufr_buffer_put_chr(buffer, c);
        }
    }
}

// ============================================================================
//  Default Encoder
// ============================================================================

int ufr_enc_sys_boot(link_t* link, const ufr_args_t* args) {
    link->enc_obj = ufr_buffer_new();
    return UFR_OK;
}

void ufr_enc_sys_close(link_t* link) {
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    if ( buffer != NULL ) {
        ufr_buffer_free(buffer);
        free(buffer);
        link->enc_obj = NULL;
    }
}

void ufr_enc_sys_clear(link_t* link) {
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    ufr_buffer_clear(buffer);
}

int ufr_enc_sys_put_u8(link_t* link, uint8_t val) {
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    ufr_buffer_put_u8_as_str(buffer, val);
    return UFR_OK;
}

int ufr_enc_sys_put_i8(link_t* link, int8_t val) {
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    ufr_buffer_put_u8_as_str(buffer, val);
    return UFR_OK;
}

int ufr_enc_sys_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    for (; wrote<nitems; wrote++) {
        ufr_buffer_put_u32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_sys_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    for (; wrote<nitems; wrote++) {
        ufr_buffer_put_i32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_sys_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    for (; wrote<nitems; wrote++) {
        ufr_buffer_put_f32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_sys_put_str(link_t* link, const char* val) {
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    const size_t size = strlen(val);
    if ( str_needs_quote(val, size) ) {
        ufr_buffer_put_chr(buffer, '\"');
        copy_str_replacing_symbols(buffer, val, size);
        ufr_buffer_put_str(buffer, "\" ");
    } else {
        copy_str_replacing_symbols(buffer, val, size);
        ufr_buffer_put_chr(buffer, ' ');
    }
    
    return UFR_OK;
}

int ufr_enc_sys_put_cmd(link_t* link, char cmd) {
    if ( cmd == '\n' ) {
        ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
        ufr_buffer_put_chr(buffer, '\n');
        ufr_write(link, buffer->ptr, buffer->size);
        ufr_buffer_clear(buffer);
    }
    return UFR_OK;
}

int ufr_enc_sys_enter(link_t* link, size_t maxsize) {
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    ufr_buffer_put_str(buffer, "[ ");
    return UFR_OK;
}

int ufr_enc_sys_leave(link_t* link) {
    ufr_buffer_t* buffer = (ufr_buffer_t*) link->enc_obj;
    ufr_buffer_put_str(buffer, "] ");
    return UFR_OK;
}

ufr_enc_api_t ufr_enc_sys_api = {
    .boot = ufr_enc_sys_boot,
    .close = ufr_enc_sys_close,
    .clear = ufr_enc_sys_clear,

    .put_u32 = ufr_enc_sys_put_u32,
    .put_i32 = ufr_enc_sys_put_i32,
    .put_f32 = ufr_enc_sys_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_enc_sys_put_cmd,
    .put_str = ufr_enc_sys_put_str,
    .put_raw = NULL,

    .enter = ufr_enc_sys_enter,
    .leave = ufr_enc_sys_leave,
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_enc_sys_new_std(link_t* link, int type) {
    link->enc_api = &ufr_enc_sys_api;
    return UFR_OK;
}