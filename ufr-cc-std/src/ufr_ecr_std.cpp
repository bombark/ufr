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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <string>
#include <errno.h>
#include <ufr.h>

struct encoder_obj_t {
    const char* sep;
    std::string line;
};

// ============================================================================
//  CSV
// ============================================================================

static
int lt_enc_csv_put_u32(link_t* link, uint32_t val) {
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
int lt_enc_csv_put_i32(link_t* link, int32_t val) {
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
int lt_enc_csv_put_f32(link_t* link, float val) {
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
int lt_enc_csv_put_str(link_t* link, const char* val) {
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
int lt_enc_csv_put_cmd(link_t* link, char cmd) {
	if ( cmd == '\n' ) {
        encoder_obj_t* enc_obj = (encoder_obj_t*) link->enc_obj;
        enc_obj->line += '\n';
        lt_write(link, enc_obj->line.c_str(), enc_obj->line.size());
        enc_obj->line.clear();
	}
	return 0;
}

static
lt_encoder_api_t lt_enc_csv = {
	.put_u32 = lt_enc_csv_put_u32,
	.put_i32 = lt_enc_csv_put_i32,
	.put_f32 = lt_enc_csv_put_f32,
	.put_str = lt_enc_csv_put_str,

	.put_cmd = lt_enc_csv_put_cmd
};

extern "C"
int ufr_new_ecr_std_csv(link_t* link, const lt_args_t* args) {
    // get parameters
	const char* sep = lt_args_gets(args, "@sep", ",");

    // 
    encoder_obj_t* enc_obj = new encoder_obj_t();
    if ( enc_obj == NULL ) {
        return lt_error(link, ENOMEM, strerror(ENOMEM));
    }
    enc_obj->sep = sep;

    // set the link
	link->enc_api = &lt_enc_csv;
	link->enc_obj = enc_obj;
	return 0;
}

