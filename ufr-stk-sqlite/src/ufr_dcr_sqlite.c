/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica Imagem (VRI)
 *   Felipe Bombardelli
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
 * */
	
// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sqlite3.h>
#include <ufr.h>

#include "ufr_gtw_sqlite.h"

// ============================================================================
//  Decoder
// ============================================================================

static
int ufr_dcr_sqlite_boot(link_t* link, const ufr_args_t* args) {
}

static
void ufr_dcr_sqlite_close(link_t* link) {
}

static
int ufr_dcr_sqlite_recv(link_t* link, char* msg_data, size_t msg_size) {

}

static
int ufr_dcr_sqlite_get_i32(link_t* link, int32_t* val, int nitems) {
    ll_gtw_obj_t* gtw_obj = link->gtw_obj;
    *val = sqlite3_column_int64(gtw_obj->stmt, gtw_obj->index);
    gtw_obj->index += 1;

	// success
	return true;
}

static
int ufr_dcr_sqlite_get_f32(link_t* link, float* val, int nitems) {
    ll_gtw_obj_t* gtw_obj = link->gtw_obj;
    *val = sqlite3_column_double(gtw_obj->stmt, gtw_obj->index);
    gtw_obj->index += 1;

	// success
	return true;
}

static
int ufr_dcr_sqlite_get_str(link_t* link, char* str, int maxbytes) {
    ll_gtw_obj_t* gtw_obj = link->gtw_obj;
	// *str = (char*) sqlite3_column_text(gtw_obj->stmt, gtw_obj->index);

	return 1;
}

static
int ufr_dcr_sqlite_copy_str(link_t* link, char* buffer, size_t size_max) {
    ll_gtw_obj_t* gtw_obj = link->gtw_obj;
	const char* text = sqlite3_column_text(gtw_obj->stmt, gtw_obj->index);
    strcpy(buffer, text);
	return 1;
}

static
int ufr_dcr_sqlite_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	

	return 1;
}

static
int ufr_dcr_sqlite_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	
	
	return UFR_OK;
}

ufr_dcr_api_t ufr_dcr_sqlite_api = {
    .boot = ufr_dcr_sqlite_boot,
    .close = ufr_dcr_sqlite_close,

    // Receive
	.recv_cb = ufr_dcr_sqlite_recv,
    .recv_async_cb = ufr_dcr_sqlite_recv,

    // ignore
    .next = NULL,

    // metadata
    .get_type = NULL,
    .get_nbytes = NULL,
    .get_nitems = NULL,
    .get_raw_ptr = NULL,

    // 32 bits
	.get_u32 = NULL,
	.get_i32 = ufr_dcr_sqlite_get_i32,
	.get_f32 = ufr_dcr_sqlite_get_f32,

    // 64 bits
	.get_u64 = NULL,
	.get_i64 = NULL,
	.get_f64 = NULL,

    // Binary and String
    .get_raw = NULL,
    .get_str = ufr_dcr_sqlite_get_str,

    // Enter and Leave
    .enter = NULL,
    .leave = NULL,
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_dcr_sqlite_new_table(link_t* link, const int type) {
    link->dcr_api = &ufr_dcr_sqlite_api;
	return 0;
}

const char* ufr_dcr_sqlite_list() {
    return "table";
}