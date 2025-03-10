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
//  Encoder
// ============================================================================

static
int ufr_enc_sqlite_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_enc_sqlite_close(link_t* link) {

}

static
int ufr_enc_sqlite_put_u32(link_t* link, uint32_t* val, int nitems) {
	ll_gtw_obj_t* obj = link->gtw_obj;
	if ( obj ) {

	}
	return 0;
}

static
int ufr_enc_sqlite_put_i32(link_t* link, int32_t* val, int nitems) {
	ll_gtw_obj_t* obj = link->gtw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_int(obj->stmt, obj->index, val[0]);
	}
	return 0;
}

static
int ufr_enc_sqlite_put_f32(link_t* link, float* val, int nitems) {
	ll_gtw_obj_t* obj = link->gtw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_double(obj->stmt, obj->index, val[0]);
	}
	return 0;
}

static
int ufr_enc_sqlite_put_str(link_t* link, const char* val) {
	ll_gtw_obj_t* obj = link->gtw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_text(obj->stmt, obj->index, val, strlen(val), NULL);
	}
	return 0;
}

static
int ufr_enc_sqlite_put_cmd(link_t* link, char cmd) {
	ll_gtw_obj_t* obj = link->gtw_obj;
    if ( cmd == '\n' ) {
        if ( sqlite3_step(obj->stmt) == SQLITE_DONE ) {
            sqlite3_reset(obj->stmt);
            obj->index = 0;
        }
    }
	return UFR_OK;
}

static
int ufr_enc_sqlite_enter(link_t* link, int nitems) {
    return -1;
}

static
int ufr_enc_sqlite_leave(link_t* link, int nitems) {
    return -1;
}

ufr_enc_api_t ufr_enc_sqlite_api = {
    .boot = ufr_enc_sqlite_boot,
    .close = ufr_enc_sqlite_close,
    .clear = NULL,

	.put_u32 = ufr_enc_sqlite_put_u32,
	.put_i32 = ufr_enc_sqlite_put_i32,
	.put_f32 = ufr_enc_sqlite_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_enc_sqlite_put_cmd,
    .put_str = ufr_enc_sqlite_put_str,
    .put_raw = NULL,

    .enter = ufr_enc_sqlite_enter,
    .leave = ufr_enc_sqlite_leave
};
