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

#include "ufr.h"

// ============================================================================
//  Buffer
// ============================================================================

ufr_buffer_t* ufr_buffer_new() {
    ufr_buffer_t* buffer = malloc(sizeof(ufr_buffer_t));
    if ( buffer != NULL ) {
        ufr_buffer_init(buffer);
    }
    return buffer;
}

void ufr_buffer_init(ufr_buffer_t* buffer) {
    buffer->size = 0;
    buffer->max = MESSAGE_ITEM_SIZE;
    buffer->ptr = malloc(buffer->max);
}

void ufr_buffer_clear(ufr_buffer_t* buffer) {
    buffer->size = 0;
    buffer->ptr[0] = '\0';
}

void ufr_buffer_free(ufr_buffer_t* buffer) {
    if ( buffer->ptr != NULL ) {
        free(buffer->ptr);
        buffer->ptr = NULL;
    }
    buffer->max = 0;
    buffer->size = 0;
}

static
void ufr_buffer_check_size(ufr_buffer_t* buffer, size_t size) {
    if ( buffer->size + size >= buffer->max ) {
        const size_t new_max = buffer->max * 2;
        char* new_ptr = realloc(buffer->ptr, new_max);
        if ( new_ptr ) {
            buffer->max = new_max;
            buffer->ptr = new_ptr;
        }
    }
}

void ufr_buffer_put(ufr_buffer_t* buffer, char* text, size_t size) {
    ufr_buffer_check_size(buffer, size);
    memcpy(&buffer->ptr[buffer->size], text, size);
    buffer->size += size;
}

void ufr_buffer_put_chr(ufr_buffer_t* buffer, char val) {
    ufr_buffer_check_size(buffer, 1);
    buffer->ptr[buffer->size] = val;
    buffer->size += 1;
}

void ufr_buffer_put_u8_as_str(ufr_buffer_t* buffer, uint8_t val) {
    ufr_buffer_check_size(buffer, 8);
    char* base = &buffer->ptr[buffer->size];
    size_t size = 0;
    if ( buffer->size == 0 ) {
        size = snprintf(base, 8, "%u", val);
    } else {
        size = snprintf(base, 8, " %u", val);
    }
    buffer->size += size;
}

void ufr_buffer_put_i8_as_str(ufr_buffer_t* buffer, int8_t val) {
    ufr_buffer_check_size(buffer, 8);
    char* base = &buffer->ptr[buffer->size];
    size_t size = 0;
    if ( buffer->size == 0 ) {
        size = snprintf(base, 8, "%u", val);
    } else {
        size = snprintf(base, 8, " %u", val);
    }
    buffer->size += size;
}

void ufr_buffer_put_u32_as_str(ufr_buffer_t* buffer, uint32_t val) {
    ufr_buffer_check_size(buffer, 32);
    char* base = &buffer->ptr[buffer->size];
    size_t size = 0;
    if ( buffer->size == 0 ) {
        size = snprintf(base, 32, "%u", val);
    } else {
        size = snprintf(base, 32, " %u", val);
    }
    buffer->size += size;
}

void ufr_buffer_put_i32_as_str(ufr_buffer_t* buffer, int32_t val) {
    ufr_buffer_check_size(buffer, 32);
    char* base = &buffer->ptr[buffer->size];
    size_t size = 0;
    if ( buffer->size == 0 ) {
        size = snprintf(base, 32, "%d", val);
    } else {
        size = snprintf(base, 32, " %d", val);
    }
    buffer->size += size;
}

void ufr_buffer_put_f32_as_str(ufr_buffer_t* buffer, float val) {
    ufr_buffer_check_size(buffer, 32);
    char* base = &buffer->ptr[buffer->size];
    size_t size = 0;
    if ( buffer->size == 0 ) {
        size = snprintf(base, 32, "%f", val);
    } else {
        size = snprintf(base, 32, " %f", val);
    }
    buffer->size += size;
}

void ufr_buffer_put_str(ufr_buffer_t* buffer, char* text) {
    const size_t size = strlen(text);
    ufr_buffer_check_size(buffer, size);
    ufr_buffer_put(buffer, text, size);
}