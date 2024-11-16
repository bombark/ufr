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

#include <stdlib.h>
#include "ufr.h"

// ============================================================================
//  GET
// ============================================================================

int ufr_get_va(link_t* link, const char* format, va_list list) {
    if ( link ) {
        if ( link->log_level > 0 ) {
            if ( link->dcr_api == NULL ) {
                ufr_fatal(link, 0, "Encoder is not loaded");
            }
            if ( link->dcr_api->get_u32 == NULL ) {
                ufr_fatal(link, 0, "Function get_u32 is NULL");
            }
            if ( link->dcr_api->get_i32 == NULL ) {
                ufr_fatal(link, -1, "Function get_i32 is NULL");
            }
            if ( link->dcr_api->get_f32 == NULL ) {
                ufr_fatal(link, -1, "Function put_f32 is NULL");
            }
            if ( link->dcr_api->get_str == NULL ) {
                ufr_fatal(link, -1, "Function put_str is NULL");
            }
        }
    } else {
        ufr_fatal(link, -1, "Link is NULL");
    }

    int retval = 0;
	while( format != NULL ) {	
		const char type = *format;
        format += 1;

		if ( type == '\0' ) {
			break;

		} else if ( type == '^' ) {
			if ( ufr_recv(link) != UFR_OK ) {
                retval = 0;
                ufr_log(link, "Error to receive data");
                break;
            }

        } else if ( type == '\n' ) {
            ufr_get_eof(link);

        } else if ( type == 'a' ) {
            const char arr_type = *format;
            format += 1;
            if ( arr_type == '\0' ) {
                break;
            }
            
            void* arr_ptr = va_arg(list, void*);
            const size_t arr_maxlen = va_arg(list, size_t);
            if ( arr_type == 'f' ) {
                // ufr_get_af32(link, arr_ptr, arr_maxlen);
            }

        } else {
            switch (type) {
                case 's': {
                    char* buffer = va_arg(list, char*);
                    if ( link->dcr_api->get_str(link, buffer, 1024) >= 0 ) {
                        retval += 1;
                    }
                } break;

                case 'u': {
                    uint32_t *val = va_arg(list, uint32_t*);
                    if ( link->dcr_api->get_u32(link, val, 1) == 1 ) {
                        retval += 1;
                    }
                } break;

                case 'i':
                case 'd': {
                    int32_t *val = va_arg(list, int32_t*);
                    if ( link->dcr_api->get_i32(link, val, 1) == 1 ) {
                        retval += 1;
                    }
                } break;

                case 'l': {
                    int64_t *val = va_arg(list, int64_t*);
                    if ( link->dcr_api->get_i64(link, val, 1) == 1 ) {
                        retval += 1;
                    }
                } break;

                case 'f': {
                    float* val = va_arg(list, float*);
                    if ( link->dcr_api->get_f32(link, val, 1) == 1 ) {
                        retval += 1;
                    }
                } break;

                case 'g': {
                    double* val = va_arg(list, double*);
                    if ( link->dcr_api->get_f64(link, val, 1) == 1 ) {
                        retval += 1;
                    }
                } break;

                case '-': {
                    link->dcr_api->next(link);
                } break;
            }
        }
	}

    // Success
    return retval;
}

int ufr_get(link_t* link, char* format, ...) {
    va_list list;
    va_start(list, format);
    const int retval = ufr_get_va(link, format, list);
    va_end(list);
    return retval;
}

void ufr_get_eof(link_t* link) {
    while (1) {
        if ( ufr_recv(link) != UFR_OK ) {
            break;
        }
    }
}

char ufr_get_type(link_t* link) {
    return link->dcr_api->get_type(link);
}


int ufr_get_raw(link_t* link, uint8_t* buffer, int max_nitems) {
    size_t arr_size = 0;
    // link->dcr_api->copy_arr(link, 'b', maxsize, &arr_size, (void*) buffer);
    return arr_size;
}

int ufr_get_f32(link_t* link, float buffer[], int max_nitems) {
    const size_t nitems = ufr_get_nitems(link);
    const size_t size = (nitems > max_nitems) ? max_nitems : nitems;
    for (size_t i=0; i<size; i++) {
        link->dcr_api->get_f32(link, buffer, max_nitems);
    }
    return size;
}




int ufr_get_nbytes(link_t* link) {
    if ( link == NULL ) {
        ufr_error(link, 0, "Link is null");
    }
    if ( link->dcr_api == NULL ) {
        ufr_error(link, 0, "Decoder of link is null");
    }
    if ( link->dcr_api->get_nbytes == NULL ) {
        ufr_error(link, 0, "Function get_size of the decoder is null");
    }
    return link->dcr_api->get_nbytes(link);
}

int ufr_get_nitems(link_t* link) {
    if ( link == NULL ) {
        ufr_error(link, 0, "Link is null");
    }
    if ( link->dcr_api == NULL ) {
        ufr_error(link, 0, "Decoder of link is null");
    }
    if ( link->dcr_api->get_nitems == NULL ) {
        ufr_error(link, 0, "Function get_size of the decoder is null");
    }
    return link->dcr_api->get_nitems(link);
}


int ufr_get_enter(link_t* link) {
    if (link->dcr_api->enter == NULL ) {
        return ufr_error(link, 1, "Function enter in Decoder is NULL");
    }
    return link->dcr_api->enter(link);
}

int ufr_get_leave(link_t* link) {
    if (link->dcr_api->leave == NULL ) {
        return ufr_error(link, 1, "Function leave in Decoder is NULL");
    }
    return link->dcr_api->leave(link);
}

const uint8_t* ufr_get_raw_ptr(link_t* link) {
    return link->dcr_api->get_raw_ptr(link);
}

bool ufr_get_str(link_t* link, char* buffer) {
    // const int is_ok = link->dcr_api->copy_str(link, buffer, -1);
    // return is_ok == UFR_OK;
    return false;
}