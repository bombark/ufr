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
#include <stdlib.h>
#include "ufr.h"

// ============================================================================
//  PUT
// ============================================================================

int ufr_put_va(link_t* link, const char* format, va_list list) {
    if ( link ) {
        if ( link->log_level > 0 ) {
            if ( link->enc_api == NULL ) {
                ufr_fatal(link, 0, "Encoder is not loaded");
            }
            if ( link->enc_api->put_cmd == NULL ) {
                ufr_fatal(link, 0, "Function put_cmd is NULL");
            }
            if ( link->enc_api->put_u32 == NULL ) {
                ufr_fatal(link, -1, "Function put_u32 is NULL");
            }
            if ( link->enc_api->put_i32 == NULL ) {
                ufr_fatal(link, -1, "Function put_i32 is NULL");
            }
            if ( link->enc_api->put_f32 == NULL ) {
                ufr_fatal(link, -1, "Function put_f32 is NULL");
            }
        }
    } else {
        ufr_fatal(link, -1, "Link is NULL");
    }

    int count = 0;
	char type;
	while( format != NULL ) {
		type = *format;
        format += 1;
		// end of string
		if ( type == '\0' ) {
			break;

		// new line
		} else if ( type == '\n' ) {
            if ( link->put_count == 0 ) {
                ufr_put_eof(link);
            } else {
			    link->enc_api->put_cmd(link, '\n');
                link->put_count = 0;
            }

		} else if ( type == 'a' ) {
            const char arr_type = *format;
            format += 1;
            if ( arr_type == '\0' ) {
                break;
            }
            const int32_t arr_size = va_arg(list, int32_t);
            if ( arr_type == 'i' ) {
                const int32_t* arr_ptr = va_arg(list, int32_t*);
                ufr_put_pi32(link, arr_ptr, arr_size);
            } else if ( arr_type == 'f' ) {
                const float* arr_ptr = va_arg(list, float*);
                ufr_put_pf32(link, arr_ptr, arr_size);
            } else if ( arr_type == 'b' ) {
                const int8_t* arr_ptr = va_arg(list, int8_t*);
                ufr_put_raw(link, arr_ptr, arr_size);
            } 

		// s, i or f
		} else {
			switch (type) {
				case 's': {
					const char* str = va_arg(list, const char*);
					link->enc_api->put_str(link, str);
				} break;
			
				case 'i':
                case 'd': {
					const int32_t val = va_arg(list, int32_t);
					link->enc_api->put_i32(link, &val, 1);
				} break;

				case 'f': {
					const float val = (float) va_arg(list, double);
					link->enc_api->put_f32(link, &val, 1);
				} break;

                // pensar se adotar essa notacao
                case '[': {
                    link->enc_api->enter(link, 100);
                } break;

                case ']': {
                    link->enc_api->leave(link);
                } break;

				default:
                    ufr_warn(link, "Operador '%c' nao definido", type);
					break;
			}
            link->put_count += 1;
            count += 1;
		}

	}
    return count;
}

int ufr_put(link_t* link, const char* format, ...) {
    va_list list;
    va_start(list, format);
    const int nitems = ufr_put_va(link, format, list);
    va_end(list);
    return nitems;
}


int ufr_put_u32(link_t* link, const uint32_t val) {
    return link->enc_api->put_u32(link, &val, 1);

}

int ufr_put_i32(link_t* link, const int32_t val) {
    return link->enc_api->put_i32(link, &val, 1);

}

int ufr_put_f32(link_t* link, const float val) {
    // check inputs
    if ( link != NULL && link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            ufr_fatal(link, 1, "Encoder is NULL");
        }
        if ( link->enc_api->put_f32 == NULL ) {
            ufr_fatal(link, 1, "enc_api->put_f32 is NULL");
        }
    }
    // send data
    return link->enc_api->put_f32(link, &val, 1);
}



int ufr_put_pu32(link_t* link, const uint32_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_u32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pi32(link_t* link, const int32_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_i32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pf32(link_t* link, const float* array, int nitems) {
    // check inputs
    if ( link != NULL && link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            ufr_fatal(link, 1, "Encoder is NULL");
        }
        if ( link->enc_api->put_f32 == NULL ) {
            ufr_fatal(link, 1, "enc_api->put_f32 is NULL");
        }
    }
    // send data
    const int wrote_nitems = link->enc_api->put_f32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

// 64bits

int ufr_put_pu64(link_t* link, const uint64_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_u64(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pi64(link_t* link, const int64_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_i64(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pf64(link_t* link, const double* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_f64(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_eof(link_t* link) {
    const int retval = link->enc_api->put_cmd(link, EOF);
    link->put_count = 0;
    if ( link->type_started == UFR_START_CLIENT ) {
        link->state = UFR_STATE_RECV;
    } else if ( link->type_started == UFR_START_SERVER ) { 
        link->state = UFR_STATE_READY;
    }
    return retval;
}

int ufr_put_str(link_t* link, const char* value) {
    return link->enc_api->put_str(link, value);
}


int ufr_put_raw(link_t* link, const uint8_t* buffer, int nitems) {
    // check inputs
    if ( link != NULL && link->enc_api->put_raw == NULL ) {
        ufr_fatal(link, 1, "enc_api->put_raw is NULL");
    }
    // send data
    const int wrote_nitems = link->enc_api->put_raw(link, buffer, nitems);
    if ( wrote_nitems > 0 ) {
        ufr_log(link, "wrote %ld bytes", nitems);
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}



int ufr_put_enter(link_t* link, int max_nitems) {
    if (link->enc_api->enter == NULL ) {
        return ufr_error(link, 1, "enter array pointer is NULL");
    }

    return link->enc_api->enter(link, max_nitems);
}

int ufr_put_leave(link_t* link) {
    if (link->enc_api->leave == NULL ) {
        return ufr_error(link, 1, "leave array pointer is NULL");
    }

    return link->enc_api->leave(link);
}