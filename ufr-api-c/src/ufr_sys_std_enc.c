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
#include "ufr.h"

// ============================================================================
//  Default Encoder
// ============================================================================

int ufr_enc_sys_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

void ufr_enc_sys_close(link_t* link) {
}

void ufr_enc_sys_clear(link_t* link) {
}

int ufr_enc_sys_set_header(link_t* link, const char* header) {
    return UFR_OK;
}


int ufr_enc_sys_put_u32(link_t* link, uint32_t val) {
    char buffer[32];
    const size_t size = snprintf(buffer, sizeof(buffer), "%u ", val);
    ufr_write(link, buffer, size);
    return UFR_OK;
}

int ufr_enc_sys_put_i32(link_t* link, int32_t val) {
    char buffer[32];
    const size_t size = snprintf(buffer, sizeof(buffer), "%d ", val);
    ufr_write(link, buffer, size);
    return UFR_OK;
}

int ufr_enc_sys_put_f32(link_t* link, float val) {
    char buffer[32];
    const size_t size = snprintf(buffer, sizeof(buffer), "%f ", val);
    ufr_write(link, buffer, size);
    return UFR_OK;
}

int ufr_enc_sys_put_str(link_t* link, const char* val) {
    const size_t size = strlen(val);
    ufr_write(link, val, size);
    return UFR_OK;
}

int ufr_enc_sys_put_cmd(link_t* link, char cmd) {
    ufr_write(link, &cmd, 1);
    return UFR_OK;
}


int ufr_enc_sys_put_arr(link_t* link, const void* array, char type, size_t size) {
    return UFR_OK;
}

int ufr_enc_sys_put_mat(link_t* link, const void* vet, char type, size_t rows, size_t cols) {
    return UFR_OK;
}


int ufr_enc_sys_enter_array(link_t* link, size_t maxsize) {
    const char c = '[';
    ufr_write(link, &c, 1);
    return UFR_OK;
}

int ufr_enc_sys_leave_array(link_t* link) {
    const char c = ']';
    ufr_write(link, &c, 1);
    return UFR_OK;
}

ufr_enc_api_t ufr_enc_sys_api = {
    .boot = ufr_enc_sys_boot,
    .close = ufr_enc_sys_close,
    .clear = ufr_enc_sys_clear,
    .set_header = ufr_enc_sys_set_header,

    .put_u32 = ufr_enc_sys_put_u32,
    .put_i32 = ufr_enc_sys_put_i32,
    .put_f32 = ufr_enc_sys_put_f32,
    .put_str = ufr_enc_sys_put_str,
    .put_cmd = ufr_enc_sys_put_cmd,

    .put_arr = ufr_enc_sys_put_arr,
    .put_mat = ufr_enc_sys_put_mat,

    .enter_array = ufr_enc_sys_enter_array,
    .leave_array = ufr_enc_sys_leave_array,
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_enc_sys_new_std(link_t* link, int type) {
    link->enc_api = &ufr_enc_sys_api;
    return UFR_OK;
}