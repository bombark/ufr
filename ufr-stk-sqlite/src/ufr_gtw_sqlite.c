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

extern ufr_enc_api_t ufr_enc_sqlite_api;
extern ufr_dcr_api_t ufr_dcr_sqlite_api;

// ============================================================================
//  Gateway
// ============================================================================

int ufr_gtw_sqlite_type(const link_t* link) {
    return 0;
}

int ufr_gtw_sqlite_state(const link_t* link) {
    return 0;
}

size_t ufr_gtw_sqlite_size(const link_t* link, int type) {
    return 0;
}

int ufr_gtw_sqlite_boot (link_t* link, const ufr_args_t* args) {
    const char* filename = ufr_args_gets(args, "@file", "sqlite3.db");
link->log_level = 5;
    // create the shared object
    ufr_log_ini(link, "opening the database");
    sqlite3 *db;
    int rc = sqlite3_open(filename, &db);
    if (rc != SQLITE_OK) {
        return ufr_log_error(link, rc, "%s", sqlite3_errmsg(db));
    }
    link->gtw_shr = (void*) db;
    ufr_log_end(link, "opened the database %s", filename);

    // create the private object
    ll_gtw_obj_t* obj = malloc(sizeof(ll_gtw_obj_t));
    obj->index = 0;
    obj->stmt = NULL;
    obj->state = 0;
    obj->rc = 0;
    obj->rc_message[0] = '\0';
    link->gtw_obj = obj;

    // success
    return UFR_OK;
}

int ufr_gtw_sqlite_start (link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_SUBSCRIBER || type == UFR_START_PUBLISHER ) {
        // close the old statment
        ll_gtw_obj_t* obj = link->gtw_obj;
        if ( obj->stmt != NULL ) {
            sqlite3_finalize(obj->stmt);
            obj->stmt = NULL;
        }

        // zero the index
        obj->index = 0;

        // get the SQL command
        const char* sql = ufr_args_gets(args, "@sql", NULL);
        if ( sql == NULL ) {
            return ufr_error(link, 1, "parameter @sql is blank");
        }

        // prepare the statment
        char const* tail;
        sqlite3 *db = link->gtw_shr;
        ufr_info(link, "sql %s", sql);
        int rc = sqlite3_prepare_v2(db, sql, strlen(sql), &obj->stmt, &tail);
        if (rc != SQLITE_OK) {
            return ufr_error(link, rc, "%s", sqlite3_errmsg(db));
        }

    }

    return UFR_OK;
}

void ufr_gtw_sqlite_stop(link_t* link, int type) {
    ll_gtw_obj_t* obj = link->gtw_obj;
    if ( obj != NULL ) {
        if ( obj->stmt != NULL ) {
            sqlite3_finalize(obj->stmt);
            obj->stmt = NULL;
        }
        free(obj);
        link->gtw_obj = NULL;
    }
}

bool ufr_gtw_sqlite_recv(link_t* link) {
    ll_gtw_obj_t* obj = link->gtw_obj;
    if ( sqlite3_step(obj->stmt) == SQLITE_ROW ) {
        obj->index = 0;
        return true;
    }
    return false;
}

bool ufr_gtw_sqlite_recv_async(link_t* link) {
    return false;
}


size_t ufr_gtw_sqlite_read(link_t* link, char* buffer, size_t max_size) {
    return 0;
}

size_t ufr_gtw_sqlite_write(link_t* link, const char* buffer, size_t size) {
    sqlite3 *db = link->gtw_shr;
    ll_gtw_obj_t* obj = link->gtw_obj;

    char const* tail;
    ufr_info(link, "sql %s", buffer);
    obj->rc = sqlite3_prepare_v2(db, buffer, size, &obj->stmt, &tail);
    if (obj->rc != SQLITE_OK) {
        strncpy(obj->rc_message, sqlite3_errmsg(db), 512);
        return ufr_error(link, 0, "%s", sqlite3_errmsg(db));
    }

    return 0;
}

static
ufr_gtw_api_t ufr_gtw_sqlite_api = {
    .name = "sqlite/client",
	.type = ufr_gtw_sqlite_type,
	.state = ufr_gtw_sqlite_state,
	.size = ufr_gtw_sqlite_size,
	.boot = ufr_gtw_sqlite_boot,
	.start = ufr_gtw_sqlite_start,
	.stop = ufr_gtw_sqlite_stop,
	.copy = NULL,
    .recv = ufr_gtw_sqlite_recv,
    .recv_async = ufr_gtw_sqlite_recv_async,
	.read = ufr_gtw_sqlite_read,
	.write = ufr_gtw_sqlite_write,
};

// ============================================================================
//  Gateway
// ============================================================================

int ufr_gtw_sqlite_client_type(const link_t* link) {
    return 0;
}

int ufr_gtw_sqlite_client_state(const link_t* link) {
    return 0;
}

size_t ufr_gtw_sqlite_client_size(const link_t* link, int type) {
    return 0;
}

int ufr_gtw_sqlite_client_boot (link_t* link, const ufr_args_t* args) {
    const char* filename = ufr_args_gets(args, "@file", "sqlite3.db");
link->log_level = 5;
    // create the shared object
    ufr_log_ini(link, "opening the database");
    sqlite3 *db;
    int rc = sqlite3_open(filename, &db);
    if (rc != SQLITE_OK) {
        return ufr_log_error(link, rc, "%s", sqlite3_errmsg(db));
    }
    link->gtw_shr = (void*) db;
    ufr_log_end(link, "opened the database %s", filename);

    // create the private object
    ll_gtw_obj_t* obj = malloc(sizeof(ll_gtw_obj_t));
    obj->index = 0;
    obj->stmt = NULL;
    obj->state = 0;
    obj->rc = 0;
    obj->rc_message[0] = '\0';
    link->gtw_obj = obj;

    // success
    return UFR_OK;
}

int ufr_gtw_sqlite_client_start (link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_SUBSCRIBER || type == UFR_START_PUBLISHER ) {
        // close the old statment
        ll_gtw_obj_t* obj = link->gtw_obj;
        if ( obj->stmt != NULL ) {
            sqlite3_finalize(obj->stmt);
            obj->stmt = NULL;
        }

        // zero the index
        obj->index = 0;
    }

    return UFR_OK;
}

void ufr_gtw_sqlite_client_stop(link_t* link, int type) {
    ll_gtw_obj_t* obj = link->gtw_obj;
    if ( obj != NULL ) {
        if ( obj->stmt != NULL ) {
            sqlite3_finalize(obj->stmt);
            obj->stmt = NULL;
        }
        // sqlite3_close(db);
        free(obj);
        link->gtw_obj = NULL;
    }
}

int ufr_gtw_sqlite_client_recv(link_t* link) {
    ll_gtw_obj_t* obj = link->gtw_obj;

    if ( obj->state == 1 ) {
        obj->state = 2;
        return UFR_OK;
    }

    /* if ( sqlite3_step(obj->stmt) == SQLITE_ROW ) {
        obj->index = 0;
        return true;
    }*/

    return -1;
}

size_t ufr_gtw_sqlite_client_read(link_t* link, char* buffer, size_t max_size) {
    ll_gtw_obj_t* obj = link->gtw_obj;
    buffer[0] = '\0';

    int wrote = 0;
    if ( obj->state == 2 ) {
        wrote = strncpy(buffer, obj->rc_message, max_size);
        obj->state = 0;
        obj->rc_message[0] = '\0';
    }

    return wrote;
}

size_t ufr_gtw_sqlite_client_write(link_t* link, const char* buffer, size_t size) {
    sqlite3* db = link->gtw_shr;
    ll_gtw_obj_t* obj = link->gtw_obj;
    if ( obj == NULL ) {
        return -1;
    }

    // Parse the first word in the buffer
    int iw = 0;
    char command[512];
    for (int ir=0; ir<512; ir++) {
        const char c = buffer[ir];
        if ( c == '\0' || c == '\n' || c == ' ' ) {
            command[iw++] = '\0';
            break;
        } else {
            command[iw++] = c | 0x20;  // convert to lower case
        }
    }

    // Command Insert
    if ( strcmp(command, "insert") == 0 ) {
        char* err_msg = NULL;
        obj->rc = sqlite3_exec(db, buffer, 0, 0, &err_msg);
        if ( obj->rc != SQLITE_OK ) {
            strncpy(obj->rc_message, err_msg, 512);
            return ufr_error(link, -1, err_msg);
        }
        const int64_t id = sqlite3_last_insert_rowid(db);
        snprintf(obj->rc_message, 512, "OK %ld", id);
        obj->state = 1;

    // Command Select
    } else if ( strcmp(command, "select") == 0 ) {

    // Command Create
    } else if ( strcmp(command, "create") == 0 ) {
        char* err_msg = NULL;
        obj->rc = sqlite3_exec(db, buffer, 0, 0, &err_msg);
        if ( obj->rc != SQLITE_OK ) {
            strncpy(obj->rc_message, err_msg, 512);
            return ufr_error(link, -1, err_msg);
        }
        strncpy(obj->rc_message, "OK", 3);
        obj->state = 1;

    // Error
    } else {
        return ufr_error(link, -1, "Command not valid");
    }

    /*
    */

    /* insert, create table, select */

    /*char* err_msg = NULL;
    const int rc = sqlite3_exec(db, buffer, 0, 0, &err_msg);
    if ( rc != SQLITE_OK ) {
        return ufr_error(link, -1, err_msg);
    }*/

    return size;
}

static
ufr_gtw_api_t ufr_gtw_sqlite_client_api = {
    .name = "sqlite/client",
	.type = ufr_gtw_sqlite_client_type,
	.state = ufr_gtw_sqlite_client_state,
	.size = ufr_gtw_sqlite_client_size,
	.boot = ufr_gtw_sqlite_client_boot,
	.start = ufr_gtw_sqlite_client_start,
	.stop = ufr_gtw_sqlite_client_stop,
	.copy = NULL,
    .recv = ufr_gtw_sqlite_client_recv,
    .recv_async = ufr_gtw_sqlite_client_recv,
	.read = ufr_gtw_sqlite_client_read,
	.write = ufr_gtw_sqlite_client_write,
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_sqlite_new(link_t* link, int type) {
    if (type == UFR_START_CLIENT ) {
        ufr_init_link(link, &ufr_gtw_sqlite_client_api);
    } else {
        ufr_fatal(link, 1, "Tipo invalido");
    }
    // link->dcr_api = &ufr_dcr_sqlite_api;
    // link->enc_api = &ufr_enc_sqlite_api;
	return UFR_OK;
}
