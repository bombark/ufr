/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Felipe Bombardelli
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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <string>
#include <ufr.h>

struct encoder_obj_t {
    const char* header;
    const char* sep;
    std::string line;
};

// ============================================================================
//  CSV
// ============================================================================

static
int ufr_enc_csv_boot(link_t* link, const ufr_args_t* args) {
    // allocate the encoder
    encoder_obj_t* enc_obj = new encoder_obj_t();
    if ( enc_obj == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    // fill the encoder data
    const char* sep = ufr_args_gets(args, "@sep", ",");
    enc_obj->sep = sep;
    link->enc_obj = enc_obj;
    return UFR_OK;
}

static
void ufr_enc_csv_close(link_t* link) {
    if ( link->enc_obj != NULL ) {
        free(link->enc_obj);
        link->enc_obj = NULL;
    }
}

static
int ufr_enc_csv_put_u32(link_t* link, uint32_t val) {
	char buffer[32];
    encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;
	const char* sep = enc_obj->sep;

	if ( enc_obj->line.size() == 0 ) {
		const size_t size = snprintf(buffer, sizeof(buffer), "%u", val);
		enc_obj->line += buffer;
	} else {
		const size_t size = snprintf(buffer, sizeof(buffer), "%s%u", sep, val);
		enc_obj->line += buffer;
	}
	
	return 0;
}

static
int ufr_enc_csv_put_i32(link_t* link, int32_t val) {
	char buffer[32];
	encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;
	const char* sep = enc_obj->sep;

	if ( enc_obj->line.size() == 0 ) {
		const size_t size = snprintf(buffer, sizeof(buffer), "%d", val);
		enc_obj->line += buffer;
	} else {
		const size_t size = snprintf(buffer, sizeof(buffer), "%s%d", sep, val);
		enc_obj->line += buffer;
	}

	return 0;
}

static
int ufr_enc_csv_put_f32(link_t* link, float val) {
	char buffer[32];
	encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;
	const char* sep = enc_obj->sep;

	if ( enc_obj->line.size() == 0 ) {
		const size_t size = snprintf(buffer, sizeof(buffer), "%f", val);
		enc_obj->line += buffer;
	} else {
		const size_t size = snprintf(buffer, sizeof(buffer), "%s%f", sep, val);
		enc_obj->line += buffer;
	}
	
	return 0;
}

static
int ufr_enc_csv_put_str(link_t* link, const char* val) {
    encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;

    if ( enc_obj->line.size() == 0 ) {
        enc_obj->line += val;
    } else {
        enc_obj->line += enc_obj->sep;
        enc_obj->line += val;
    }

    return 0;
}

static
int ufr_enc_csv_put_cmd(link_t* link, char cmd) {
    if ( cmd == '\n' ) {
        encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;
        enc_obj->line += '\n';
        ufr_write(link, enc_obj->line.c_str(), enc_obj->line.size());
        ufr_send(link);
        enc_obj->line.clear();
    }
    return 0;
}

int ufr_enc_enter_array(link_t* link, size_t maxsize) {
    encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;
    if ( enc_obj->line.size() > 0 ) {
        enc_obj->line += enc_obj->sep;
    }
    enc_obj->line += "#ENTER_ARRAY";
    return UFR_OK;
}


int ufr_enc_leave_array(link_t* link) {
    encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;
    if ( enc_obj->line.size() > 0 ) {
        enc_obj->line += enc_obj->sep;
    }
    enc_obj->line += "#LEAVE_ARRAY";
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_std_csv_api = {
    .boot = ufr_enc_csv_boot,
    .close = ufr_enc_csv_close,

	.put_u32 = ufr_enc_csv_put_u32,
	.put_i32 = ufr_enc_csv_put_i32,
	.put_f32 = ufr_enc_csv_put_f32,
	.put_str = ufr_enc_csv_put_str,

	.put_cmd = ufr_enc_csv_put_cmd,

    .enter_array = ufr_enc_enter_array,
    .leave_array = ufr_enc_leave_array
};

// ============================================================================
//  Public Funtions
// ============================================================================

extern "C"
int ufr_enc_csv_new(link_t* link, int type) {
	link->enc_api = &ufr_enc_std_csv_api;
	return UFR_OK;
}

