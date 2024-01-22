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

typedef struct  {
    size_t index;
    sqlite3_stmt* stmt;
} ll_gw_obj_t;


/*
static int lt_gw_callback(void* data, int argc, char** argv, char** azColName) {

    for (int i = 0; i < argc; i++) { 
        printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL"); 
    }

    return 0;
}
*/

// ============================================================================
//  Decoder
// ============================================================================

static
void lt_dec_sqlite_recv(link_t* link, char* msg_data, size_t msg_size) {

}

static
int lt_dec_sqlite_get_i32(link_t* link, int32_t* val) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
    *val = sqlite3_column_int64(gw_obj->stmt, gw_obj->index);
    gw_obj->index += 1;

	// success
	return true;
}

static
int lt_dec_sqlite_get_f32(link_t* link, float* val) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
    *val = sqlite3_column_double(gw_obj->stmt, gw_obj->index);
    gw_obj->index += 1;

	// success
	return true;
}

static
int lt_dec_sqlite_get_str(link_t* link, char** str) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
	*str = (char*) sqlite3_column_text(gw_obj->stmt, gw_obj->index);

	return 1;
}

static
int lt_dec_sqlite_copy_str(link_t* link, char* buffer, size_t size_max) {
    ll_gw_obj_t* gw_obj = link->gw_obj;
	const char* text = sqlite3_column_text(gw_obj->stmt, gw_obj->index);
    strcpy(buffer, text);
	return 1;
}

static
int lt_dec_sqlite_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	

	return 1;
}

static
int lt_dec_sqlite_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
	
	
	return LT_OK;
}

lt_decoder_api_t lt_dec_sqlite = {
	.recv = lt_dec_sqlite_recv,

	.get_u32 = NULL,
	.get_i32 = lt_dec_sqlite_get_i32,
	.get_f32 = lt_dec_sqlite_get_f32,
	.get_str = lt_dec_sqlite_get_str,
    .copy_str = lt_dec_sqlite_copy_str,
	.get_arr = lt_dec_sqlite_get_arr,
	.copy_arr = lt_dec_sqlite_copy_arr
};

// ============================================================================
//  Encoder
// ============================================================================

static
int lt_enc_sqlite_put_u32(link_t* link, uint32_t val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {

	}
	return 0;
}

static
int lt_enc_sqlite_put_i32(link_t* link, int32_t val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_int(obj->stmt, obj->index, val);
	}
	return 0;
}

static
int lt_enc_sqlite_put_f32(link_t* link, float val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_double(obj->stmt, obj->index, val);
	}
	return 0;
}

static
int lt_enc_sqlite_put_str(link_t* link, const char* val) {
	ll_gw_obj_t* obj = link->gw_obj;
	if ( obj ) {
        obj->index += 1;
        sqlite3_bind_text(obj->stmt, obj->index, val, strlen(val), NULL);
	}
	return 0;
}

static
int lt_enc_sqlite_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
	ll_gw_obj_t* obj = link->gw_obj;

}

static
int lt_enc_sqlite_put_cmd(link_t* link, char cmd) {
	ll_gw_obj_t* obj = link->gw_obj;
    if ( cmd == '\n' ) {
        if ( sqlite3_step(obj->stmt) == SQLITE_DONE ) {
            sqlite3_reset(obj->stmt);
            obj->index = 0;
        }
    }
	return LT_OK;
}

static
lt_encoder_api_t lt_enc_sqlite = {
	.put_u32 = lt_enc_sqlite_put_u32,
	.put_i32 = lt_enc_sqlite_put_i32,
	.put_f32 = lt_enc_sqlite_put_f32,
	.put_str = lt_enc_sqlite_put_str,
	.put_arr = lt_enc_sqlite_put_arr,
	.put_cmd = lt_enc_sqlite_put_cmd
};

// ============================================================================
//  Gateway
// ============================================================================

int lt_gw_sqlite_type(const link_t* link) {
    return 0;
}

int lt_gw_sqlite_state(const link_t* link) {
    return 0;
}

size_t lt_gw_sqlite_size(const link_t* link, int type) {
    return 0;
}

int lt_gw_sqlite_boot (link_t* link, const lt_args_t* args) {
    const char* filename = lt_args_gets(args, "@file", "sqlite3.db");

    // create the shared object
    sqlite3 *db;
    int rc = sqlite3_open(filename, &db);
    if (rc != SQLITE_OK) {
        lt_error(link, rc, "%s", sqlite3_errmsg(db));
    }
    link->gw_shr = (void*) db;

    // create the private object
    ll_gw_obj_t* obj = malloc(sizeof(ll_gw_obj_t));
    obj->index = 0;
    obj->stmt = NULL;
    link->gw_obj = obj;

    // success
    return LT_OK;
}

int lt_gw_sqlite_start (link_t* link, int type, const lt_args_t* args) {
    if ( type == LT_START_SUBSCRIBER || type == LT_START_PUBLISHER ) {
        // close the old statment
        ll_gw_obj_t* obj = link->gw_obj;
        if ( obj->stmt != NULL ) {
            sqlite3_finalize(obj->stmt);
            obj->stmt = NULL;
        }

        // zero the index
        obj->index = 0;

        // get the SQL command
        const char* sql = lt_args_gets(args, "@sql", NULL);
        if ( sql == NULL ) {
            return lt_error(link, 1, "parameter @sql is blank");
        }

        // prepare the statment
        char const* tail;
        sqlite3 *db = link->gw_shr;
        lt_info(link, "sql %s", sql);
        int rc = sqlite3_prepare_v2(db, sql, strlen(sql), &obj->stmt, &tail);
        if (rc != SQLITE_OK) {
            return lt_error(link, rc, "%s", sqlite3_errmsg(db));
        }

    }

    return LT_OK;
}

void lt_gw_sqlite_stop(link_t* link, int type) {
    ll_gw_obj_t* obj = link->gw_obj;
    if ( obj != NULL ) {
        if ( obj->stmt != NULL ) {
            sqlite3_finalize(obj->stmt);
            obj->stmt = NULL;
        }
        free(obj);
        link->gw_obj = NULL;
    }
}

void lt_gw_sqlite_recv(link_t* link) {
    ll_gw_obj_t* obj = link->gw_obj;
    if ( sqlite3_step(obj->stmt) == SQLITE_ROW ) {
        obj->index = 0;
    }
}

bool lt_gw_sqlite_recv_async(link_t* link) {
    return false;
}


size_t lt_gw_sqlite_read(link_t* link, char* buffer, size_t max_size) {
    return 0;
}

size_t lt_gw_sqlite_write(link_t* link, const char* buffer, size_t size) {
    sqlite3 *db = link->gw_shr;
    ll_gw_obj_t* obj = link->gw_obj;

    char const* tail;
    lt_info(link, "sql %s", buffer);
    int rc = sqlite3_prepare_v2(db, buffer, size, &obj->stmt, &tail);
    if (rc != SQLITE_OK) {
        return lt_error(link, 0, "%s", sqlite3_errmsg(db));
    }

    return 0;
}

static
lt_api_t lt_sqlite_table = {
	.type = lt_gw_sqlite_type,
	.state = lt_gw_sqlite_state,
	.size = lt_gw_sqlite_size,
	.boot = lt_gw_sqlite_boot,
	.start = lt_gw_sqlite_start,
	.stop = lt_gw_sqlite_stop,
	.copy = NULL,
    .recv = lt_gw_sqlite_recv,
    .recv_async = lt_gw_sqlite_recv_async,
	.read = lt_gw_sqlite_read,
	.write = lt_gw_sqlite_write,
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_new_gtw_sqlite_table(link_t* link, const lt_args_t* args) {
	link->gw_api = &lt_sqlite_table;
    link->dec_api = &lt_dec_sqlite;
    link->enc_api = &lt_enc_sqlite;
	return lt_gw_sqlite_boot(link, args);
}

const char* lt_gw_sqlite_list() {
    return "file";
}
