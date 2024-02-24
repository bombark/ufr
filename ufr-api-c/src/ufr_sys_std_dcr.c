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
#include "ufr.h"

// ============================================================================
//  Default Decoder
// ============================================================================

static
int ufr_dcr_sys_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_dcr_sys_close(link_t* link) {

}

static
void ufr_dcr_sys_recv(link_t* link, char* msg_data, size_t msg_size) {

}

static
int ufr_dcr_sys_get_u32(link_t* link, uint32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_sys_get_i32(link_t* link, int32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_sys_get_f32(link_t* link, float* ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_sys_get_str(link_t* link, char** ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_sys_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_sys_copy_str(link_t* link, char* ret_val, size_t size_max) {
    return UFR_OK;
}

static
int ufr_dcr_sys_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_sys_enter_array(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_sys_leave_array(link_t* link) {
    return UFR_OK;
}

static
ufr_dcr_api_t dcr_sys_api = {
    .boot = ufr_dcr_sys_boot,
    .close = ufr_dcr_sys_close,
	.recv = ufr_dcr_sys_recv,

	.get_u32 = ufr_dcr_sys_get_u32,
	.get_i32 = ufr_dcr_sys_get_i32,
	.get_f32 = ufr_dcr_sys_get_f32,
	.get_str = ufr_dcr_sys_get_str,
	.get_arr = ufr_dcr_sys_get_arr,

    .copy_str = ufr_dcr_sys_copy_str,
	.copy_arr = ufr_dcr_sys_copy_arr,

    .enter_array = ufr_dcr_sys_enter_array,
    .leave_array = ufr_dcr_sys_leave_array
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_dcr_sys_new_std(link_t* link, int type) {
    link->dcr_api = &dcr_sys_api;
    return UFR_OK;
}