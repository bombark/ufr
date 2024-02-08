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
void ufr_dcr_sqlite_recv(link_t* link, char* msg_data, size_t msg_size) {

}

static
int ufr_dcr_sqlite_get_i32(link_t* link, int32_t* val) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
    *val = sqlite3_column_int64(gw_obj->stmt, gw_obj->index);
    gw_obj->index += 1;

	// success
	return true;
}

static
int ufr_dcr_sqlite_get_f32(link_t* link, float* val) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
    *val = sqlite3_column_double(gw_obj->stmt, gw_obj->index);
    gw_obj->index += 1;

	// success
	return true;
}

static
int ufr_dcr_sqlite_get_str(link_t* link, char** str) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
	*str = (char*) sqlite3_column_text(gw_obj->stmt, gw_obj->index);

	return 1;
}

static
int ufr_dcr_sqlite_copy_str(link_t* link, char* buffer, size_t size_max) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
	const char* text = sqlite3_column_text(gw_obj->stmt, gw_obj->index);
    strcpy(buffer, text);
	return 1;
}

static
int ufr_dcr_sqlite_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	

	return 1;
}

static
int ufr_dcr_sqlite_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	
	
	return LT_OK;
}

lt_decoder_api_t ufr_dcr_sqlite_api = {
	.recv = ufr_dcr_sqlite_recv,

	.get_u32 = NULL,
	.get_i32 = ufr_dcr_sqlite_get_i32,
	.get_f32 = ufr_dcr_sqlite_get_f32,
	.get_str = ufr_dcr_sqlite_get_str,
    .copy_str = ufr_dcr_sqlite_copy_str,
	.get_arr = ufr_dcr_sqlite_get_arr,
	.copy_arr = ufr_dcr_sqlite_copy_arr
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