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

typedef struct {
    char* msg_ptr;
    size_t msg_size;
    char* msg_end;
    char* msg_cur;
} decoder_t;

// ============================================================================
//  Default Decoder
// ============================================================================

static
int ufr_dcr_sys_boot(link_t* link, const ufr_args_t* args) {
    decoder_t* dcr = malloc(sizeof(decoder_t));
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_sys_close(link_t* link) {
    if ( link->dcr_obj != NULL ) {
        free(link->dcr_obj);
        link->dcr_obj = NULL;
    }
}

static
int ufr_dcr_sys_recv(link_t* link, char* msg_data, size_t msg_size) {
    decoder_t* dcr = link->dcr_obj;
    if ( dcr != NULL ) {
        dcr->msg_ptr = msg_data;
        dcr->msg_size = msg_size;
        dcr->msg_cur = msg_data;
        dcr->msg_end = msg_data+msg_size;
        return UFR_OK;
    }
    
    return -1;
}

static
int ufr_dcr_sys_get_type(link_t* link) {
    return 0;
}

static
int ufr_dcr_sys_get_u32(link_t* link, uint32_t* val, int nitems) {
    decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL || dcr->msg_cur >= dcr->msg_end ) {
        return -1;
    }

    char* saveptr;
    int wrote = 0;
    for (;wrote<nitems; wrote++) {
        const char* val_str = strtok_r( dcr->msg_cur, " \n", &saveptr );
        if ( val_str == NULL ) {
            break;
        }
        sscanf(val_str, "%u", val);
    }
    dcr->msg_cur = saveptr;
    return UFR_OK;
}

static
int ufr_dcr_sys_get_i32(link_t* link, int32_t* val, int nitems) {
    decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL || dcr->msg_cur >= dcr->msg_end ) {
        return -1;
    }

    char* saveptr;
    int wrote = 0;
    for (;wrote<nitems; wrote++) {
        const char* val_str = strtok_r( dcr->msg_cur, " \n", &saveptr );
        if ( val_str == NULL ) {
            break;
        }
        sscanf(val_str, "%d", val);
    }
    dcr->msg_cur = saveptr;
    return wrote;
}

static
int ufr_dcr_sys_get_f32(link_t* link, float* val, int nitems) {
    decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL || dcr->msg_cur >= dcr->msg_end ) {
        return -1;
    }

    char* saveptr;
    int wrote = 0;
    for (;wrote<nitems; wrote++) {
        const char* val_str = strtok_r( dcr->msg_cur, " \n", &saveptr );
        if ( val_str == NULL ) {
            break;
        }
        sscanf(val_str, "%f", val);
    }
    dcr->msg_cur = saveptr;
    return wrote;
}

static
int ufr_dcr_sys_get_str(link_t* link, char* ret_val, int maxbytes) {
    ret_val[0] = '\0';

    decoder_t* dcr = link->dcr_obj;
    if ( dcr == NULL || dcr->msg_cur >= dcr->msg_end ) {
        return -1;
    }

    char* saveptr;
    const char* val_str = strtok_r( dcr->msg_cur, " \n", &saveptr );
    if ( val_str == NULL ) {
        return -1;
    }
    strcpy(ret_val, val_str);
    dcr->msg_cur = saveptr;
    return UFR_OK;
}

static
int ufr_dcr_sys_enter(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_sys_leave(link_t* link) {
    return UFR_OK;
}

static
ufr_dcr_api_t dcr_sys_api = {
    .boot = ufr_dcr_sys_boot,
    .close = ufr_dcr_sys_close,
	.recv_cb = ufr_dcr_sys_recv,
    .recv_async_cb = NULL,

    .next = NULL, 

    .get_type = NULL,

	.get_u32 = ufr_dcr_sys_get_u32,
	.get_i32 = ufr_dcr_sys_get_i32,
	.get_f32 = ufr_dcr_sys_get_f32,

    .get_raw = NULL,
    .get_str = ufr_dcr_sys_get_str,

    .enter = ufr_dcr_sys_enter,
    .leave = ufr_dcr_sys_leave
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_dcr_sys_new_std(link_t* link, int type) {
    link->dcr_api = &dcr_sys_api;
    return UFR_OK;
}