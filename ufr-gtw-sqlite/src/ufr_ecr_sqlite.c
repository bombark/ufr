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
int ufr_ecr_sqlite_put_u32(link_t* link, uint32_t val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {

	}
	return 0;
}

static
int ufr_ecr_sqlite_put_i32(link_t* link, int32_t val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_int(obj->stmt, obj->index, val);
	}
	return 0;
}

static
int ufr_ecr_sqlite_put_f32(link_t* link, float val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_double(obj->stmt, obj->index, val);
	}
	return 0;
}

static
int ufr_ecr_sqlite_put_str(link_t* link, const char* val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_text(obj->stmt, obj->index, val, strlen(val), NULL);
	}
	return 0;
}

static
int ufr_ecr_sqlite_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
	ll_gw_obj_t* obj = link->gw_obj;

}

static
int ufr_ecr_sqlite_put_cmd(link_t* link, char cmd) {
	ll_gw_obj_t* obj = link->gw_obj;
    if ( cmd == '\n' ) {
        if ( sqlite3_step(obj->stmt) == SQLITE_DONE ) {
            sqlite3_reset(obj->stmt);
            obj->index = 0;
        }
    }
	return LT_OK;
}

lt_encoder_api_t ufr_ecr_sqlite_api = {
	.put_u32 = ufr_ecr_sqlite_put_u32,
	.put_i32 = ufr_ecr_sqlite_put_i32,
	.put_f32 = ufr_ecr_sqlite_put_f32,
	.put_str = ufr_ecr_sqlite_put_str,
	.put_arr = ufr_ecr_sqlite_put_arr,
	.put_cmd = ufr_ecr_sqlite_put_cmd
};
