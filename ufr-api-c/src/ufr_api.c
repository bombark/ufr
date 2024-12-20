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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ufr.h"

uint8_t g_default_log_level = 2;
volatile bool g_is_ok = true;

typedef int (*loop_callback)(void);
uint8_t g_callback_array_count = 0;
loop_callback g_callback_array[5] = {NULL, NULL, NULL, NULL, NULL};

// ============================================================================
//  Link - Meta
// ============================================================================

int ufr_put_loop_callback( int (*loop_callback)(void)  ) {
    if ( g_callback_array_count < 5 ) {
        g_callback_array[ g_callback_array_count ] = loop_callback;
        g_callback_array_count += 1;
        return UFR_OK;
    }
    return -1;
}

bool ufr_loop_ok() {
    for (uint8_t i=0; i<g_callback_array_count; i++) {
        if ( g_callback_array[i]() != UFR_OK ) {
            return false;
        }
    }
    return true;
}



const char* ufr_api_name(const link_t* link) {
	if ( link == NULL || link->gtw_api == NULL ) {
		return "None";
	}
	return link->gtw_api->name;
}

int ufr_link_state(const link_t* link) {
    return link->type_started;
}

bool ufr_link_is_publisher(const link_t* link) {
    return link->type_started == UFR_START_PUBLISHER;
}

bool ufr_link_is_subscriber(const link_t* link) {
    return link->type_started == UFR_START_SUBSCRIBER;
}

bool ufr_link_is_server(const link_t* link) {
    return link->type_started == UFR_START_SERVER;
}

bool ufr_link_is_client(const link_t* link) {
    return link->type_started == UFR_START_CLIENT;
}

void ufr_init_link(link_t* link, ufr_gtw_api_t* gtw_api) {
    link->gtw_api = gtw_api;
    link->gtw_obj = NULL;
    link->gtw_shr = NULL;
    link->dcr_api = NULL;
    link->dcr_obj = NULL;
    link->enc_api = NULL;
    link->enc_obj = NULL;
    link->type_started = UFR_START_BLANK;
    link->status = UFR_STATUS_RESET;
    link->put_count = 0;
    // pensar sobre isso
    // - link->log_level = g_default_log_level;
    // - link->log_ident = 0;
}

int ufr_boot_dcr(link_t* link, const ufr_args_t* args) {
    ufr_log_ini(link, "booting decoder");
    const int state = link->dcr_api->boot(link, args);
    ufr_log_end(link, "decoder booted");
    return state;
}

int ufr_boot_enc(link_t* link, const ufr_args_t* args) {
    ufr_log_ini(link, "booting encoder");
    const int state = link->enc_api->boot(link, args);
    ufr_log_end(link, "encoder booted");
    return state;
}

int ufr_boot_gtw(link_t* link, const ufr_args_t* args) {
    ufr_log_ini(link, "booting gateway");
    const int state = link->gtw_api->boot(link, args);
    if ( state == UFR_OK ) {
        link->status = UFR_STATUS_BOOTED;
        link->is_booted = 1;
    }
    ufr_log_end(link, "gateway booted");
    return state;
}

int ufr_boot_subscriber(link_t* link, const char* text) {
    if ( link == NULL || link->gtw_api == NULL ) {
        return 1;
    }

    const ufr_args_t args = {.text=text};
    int retval = ufr_boot_gtw(link, &args);
    if ( retval == UFR_OK ) {
        if ( link->dcr_api != NULL ) {
            retval = ufr_boot_dcr(link, &args);
        }
    }

    if ( retval == UFR_OK ) {
        retval = ufr_start(link, UFR_START_SUBSCRIBER, &args);
    }

    return retval;
}

int ufr_boot_publisher(link_t* link, const char* text) {
    if ( link == NULL || link->gtw_api == NULL ) {
        return 1;
    }
    link->log_ident = 0;

    const ufr_args_t args = {.text=text};
    int retval = ufr_boot_gtw(link, &args);
    if ( retval == UFR_OK ) {
        if ( link->enc_api != NULL ) {
            retval = ufr_boot_enc(link, &args);
        }
    }

    if ( retval == UFR_OK ) {
        retval = ufr_start(link, UFR_START_PUBLISHER, &args);
    }

    return retval;
}

int ufr_boot_server(link_t* link, const char* text) {
    if ( link == NULL || link->gtw_api == NULL ) {
        return 1;
    }

    const ufr_args_t args = {.text=text};
    int retval = ufr_boot_gtw(link, &args);
    if ( retval == UFR_OK && link->dcr_api != NULL ) {
        retval = ufr_boot_dcr(link, &args);
    }
    if ( retval == UFR_OK && link->enc_api != NULL ) {
        retval = ufr_boot_enc(link, &args);
    }
    return retval;
}


int ufr_start(link_t* link, int type, const ufr_args_t* param_args) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->start == NULL ) {
            ufr_fatal(link, 1, "gtw_api->start is null");
        }
    }

    ufr_log_ini(link, "starting link");

    if ( link->status == UFR_STATUS_RESET ) {
        if ( ufr_boot_gtw(link, param_args) != UFR_OK ) {
            return 1;
        }
    }

    // select the arguments avoiding NULL pointer
    const ufr_args_t empty_args = {.text=""};
    const ufr_args_t* args = ( param_args != NULL ) ? param_args : &empty_args;

    // call driver function
    const int error = link->gtw_api->start(link, type, args);
    if ( error != UFR_OK ) {
        return ufr_log_error(link, error, "error");
    }

    // done
    link->type_started = type;
    link->status = UFR_STATUS_STARTED;
    link->is_started = 1;
    ufr_log_end(link, "link started");
    return UFR_OK;
}

int ufr_start_publisher(link_t* link, const ufr_args_t* args) {
    return ufr_start(link, UFR_START_PUBLISHER, args);
}

int ufr_start_subscriber(link_t* link, const ufr_args_t* args) {
    return ufr_start(link, UFR_START_SUBSCRIBER, args);
}

int ufr_start_server(link_t* link, const ufr_args_t* args) {
    return ufr_start(link, UFR_START_SERVER, args);
}

int ufr_start_client(link_t* link, const ufr_args_t* args) {
    return ufr_start(link, UFR_START_CLIENT, args);
}

int ufr_recv(link_t* link) {
    ufr_log_ini(link, "receiving data from link");
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->recv == NULL ) {
            ufr_fatal(link, 1, "gtw_api->recv is null");
        }
    }
    
    link->state = 1;
    const int retval = link->gtw_api->recv(link);
    ufr_log_end(link, "received data from link");
    return retval;
}

int ufr_recv_async(link_t* link) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->recv_async == NULL ) {
            ufr_fatal(link, 1, "gtw_api->recv_async is null");
        }
    }

    return link->gtw_api->recv_async(link);
}

bool ufr_send(link_t* link) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->enc_api->put_cmd == NULL ) {
            ufr_fatal(link, 1, "enc_api->put_cmd is null");
        }
    }

    int error = link->enc_api->put_cmd(link, '\n');
    link->put_count = 0;
    return error == UFR_OK;
}

void ufr_close(link_t* link) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->stop == NULL ) {
            ufr_fatal(link, 1, "gtw_api->stop is null");
        }
    }

    link->gtw_api->stop(link, UFR_STOP_CLOSE);
    link->is_started = 0;
    link->is_booted = 0;
}

// ============================================================================
//  Link - Character Stream
// ============================================================================

size_t ufr_read(link_t* link, char* buffer, size_t maxsize) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->read == NULL ) {
            ufr_fatal(link, 1, "gtw_api->read is null");
        }
    }

	return link->gtw_api->read(link, buffer, maxsize);
}

size_t ufr_write(link_t* link, const char* buffer, size_t size) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->read == NULL ) {
            ufr_fatal(link, 1, "gtw_api->write is null");
        }
    }

	return link->gtw_api->write(link, buffer, size);
}

// ============================================================================
//  Link - Formatted Stream
// ============================================================================

int ufr_get_va(link_t* link, const char* format, va_list list) {
    if ( link->dcr_api == NULL ) {
        ufr_error(link, 0, "Decoder is not loaded");
        return -1;
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
                ufr_get_af32(link, arr_ptr, arr_maxlen);
            }

        } else {
            switch (type) {
                case 's': {
                    char* buffer = va_arg(list, char*);
                    if ( link->dcr_api->get_str(link, buffer, 1024) != -1 ) {
                        retval += 1;
                    }
                } break;

                case 'i': {
                    int32_t *val = va_arg(list, int32_t*);
                    if ( link->dcr_api->get_i32(link, val) == UFR_OK ) {
                        retval += 1;
                    }
                } break;

                case 'f': {
                    float* val = va_arg(list, float*);
                    if ( link->dcr_api->get_f32(link, val) == UFR_OK ) {
                        retval += 1;
                    }
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

size_t ufr_get_size(link_t* link) {
    if ( link == NULL ) {
        ufr_error(link, 0, "Link is null");
    }
    if ( link->dcr_api == NULL ) {
        ufr_error(link, 0, "Decoder of link is null");
    }
    if ( link->dcr_api->get_size == NULL ) {
        ufr_error(link, 0, "Function get_size of the decoder is null");
    }
    return link->dcr_api->get_size(link);
}

const uint8_t* ufr_get_raw_ptr(link_t* link) {
    return link->dcr_api->get_raw_ptr(link);
}

bool ufr_get_str(link_t* link, char* buffer) {
    // const int is_ok = link->dcr_api->copy_str(link, buffer, -1);
    // return is_ok == UFR_OK;
    return false;
}

void ufr_put_va(link_t* link, const char* format, va_list list) {
    if ( link->enc_api == NULL ) {
        ufr_error(link, 0, "Encoder is not loaded");
        return;
    }

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
                ufr_put_ai32(link, arr_ptr, arr_size);
            } else if ( arr_type == 'f' ) {
                const float* arr_ptr = va_arg(list, float*);
                ufr_put_af32(link, arr_ptr, arr_size);
            } else if ( arr_type == 'b' ) {
                const int8_t* arr_ptr = va_arg(list, int8_t*);
                ufr_put_ai8(link, arr_ptr, arr_size);
            } 

		// s, i or f
		} else {
			switch (type) {
				case 's': {
					const char* str = va_arg(list, const char*);
					link->enc_api->put_str(link, str);
				} break;
			
				case 'i': {
					const int32_t val = va_arg(list, int32_t);
					link->enc_api->put_i32(link, val);
				} break;

				case 'f': {
					const float val = (float) va_arg(list, double);
					link->enc_api->put_f32(link, val);
				} break;

                // pensar se adotar essa notacao
                case '[': {
                    link->enc_api->enter_array(link, 100);
                } break;

                case ']': {
                    link->enc_api->leave_array(link);
                } break;

				default:
                    ufr_warn(link, "Operador '%c' nao definido", type);
					break;
			}
            link->put_count += 1;
		}

	}
	
}

void ufr_put(link_t* link, const char* format, ...) {
    va_list list;
    va_start(list, format);
    ufr_put_va(link, format, list);
    va_end(list);
}

void ufr_put_au8(link_t* link, const uint8_t* array, size_t size) {
    link->put_count += 1;
    ufr_enter_array(link, size);
    for (size_t i=0; i<size; i++) {
        link->enc_api->put_u8(link, array[i]);
    }
    ufr_leave_array(link);
}

void ufr_put_ai8(link_t* link, const int8_t* array, size_t size) {
    link->put_count += 1;
    ufr_enter_array(link, size);
    for (size_t i=0; i<size; i++) {
        link->enc_api->put_i8(link, array[i]);
    }
    ufr_leave_array(link);
}

void ufr_put_au32(link_t* link, const uint32_t* array, size_t size) {
    link->put_count += 1;
    ufr_enter_array(link, size);
    for (size_t i=0; i<size; i++) {
        link->enc_api->put_u32(link, array[i]);
    }
    ufr_leave_array(link);
}

void ufr_put_ai32(link_t* link, const int32_t* array, size_t size) {
    link->put_count += 1;
    ufr_enter_array(link, size);
    for (size_t i=0; i<size; i++) {
        link->enc_api->put_i32(link, array[i]);
    }
    ufr_leave_array(link);
}

void ufr_put_af32(link_t* link, const float* array, size_t size) {
    link->put_count += 1;
    ufr_enter_array(link, size);
    for (size_t i=0; i<size; i++) {
        link->enc_api->put_f32(link, array[i]);
    }
    ufr_leave_array(link);
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

void ufr_put_raw(link_t* link, const uint8_t* buffer, size_t size) {
    link->put_count += 1;   
    link->enc_api->put_raw(link, buffer, size);
}

size_t ufr_get_raw(link_t* link, uint8_t* buffer, size_t maxsize) {
    size_t arr_size = 0;
    // link->dcr_api->copy_arr(link, 'b', maxsize, &arr_size, (void*) buffer);
    return arr_size;
}

size_t ufr_get_af32(link_t* link, float buffer[], size_t maxlen) {
    const size_t arr_size = ufr_get_size(link);
    const size_t size = (arr_size > maxlen) ? maxlen : arr_size;
    ufr_dcr_enter(link);
    for (size_t i=0; i<size; i++) {
        link->dcr_api->get_f32(link, &buffer[i]);
    }
    ufr_dcr_leave(link);
    return size;
}



int ufr_enter_array(link_t* link, size_t arr_size_max) {
    if (link->enc_api->enter_array == NULL ) {
        return ufr_error(link, 1, "enter array pointer is NULL");
    }

    return link->enc_api->enter_array(link, arr_size_max);
}

int ufr_leave_array(link_t* link) {
    if (link->enc_api->enter_array == NULL ) {
        return ufr_error(link, 1, "leave array pointer is NULL");
    }

    return link->enc_api->leave_array(link);
}

int ufr_dcr_enter(link_t* link) {
    if (link->dcr_api->enter == NULL ) {
        return ufr_error(link, 1, "Function enter in Decoder is NULL");
    }
    return link->dcr_api->enter(link);
}

int ufr_dcr_leave(link_t* link) {
    if (link->dcr_api->leave == NULL ) {
        return ufr_error(link, 1, "Function leave in Decoder is NULL");
    }
    return link->dcr_api->leave(link);
}

// ============================================================================
//  Arguments
// ============================================================================

bool ufr_flex_text_div(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max, const char div) {
    uint8_t state = 0;
    uint16_t i_token = 0;
    uint16_t i_text = *cursor_ini;
    token[0] = '\0';
    while (1) {
        const char c = text[i_text];
        if ( c == '\0' ) {
            token[i_token] = '\0';
            break;
        }
        
        // ignore caracter
        if ( c == '\n' ) {
            i_text += 1;
            continue;
        }

        // standard state
        if ( state == 0 ) {
        
            if ( c == '\'') {
                state = 1;

            } else if ( c == div ) {
                if ( i_token > 0 ) {
                    token[i_token] = '\0';
                    break;
                }

            } else {
                if ( i_token < token_max-1 ) {
                    token[i_token] = c;
                    i_token += 1;
                }
            }

        // inside quotes, example: 'text'
        } else if ( state == 1 ) {
            if ( c == '\'') {
                state = 0;
            } else {
                if ( i_token < token_max-1 ) {
                    token[i_token] = c;
                    i_token += 1;
                }
            }
        }

        i_text += 1;
    }

    *cursor_ini = i_text;
    return (i_token > 0);
}

// static
bool ufr_flex_text(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max) {
    return ufr_flex_text_div(text, cursor_ini, token, token_max, ' ');
}

size_t ufr_args_getu(const ufr_args_t* args, const char* noun, const size_t default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_flex_text(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'd' ) {
                    return args->arg[count_arg].i32;
                } else if ( token[1] == 's' ) {
                    return atoi(args->arg[count_arg].str);
                } else if ( token[1] == 'f' ) {
                    return args->arg[count_arg].f32;
                }
            } else {
                return atoi(token);
            }
        }
    }

    // not found, return default value
    return default_value;
}

int ufr_args_geti(const ufr_args_t* args, const char* noun, const int default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_flex_text(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'd' ) {
                    return args->arg[count_arg].i32;
                } else if ( token[1] == 's' ) {
                    return atoi(args->arg[count_arg].str);
                } else if ( token[1] == 'f' ) {
                    return args->arg[count_arg].f32;
                }
            } else {
                return atoi(token);
            }
        }
    }

    // not found, return default value
    return default_value;
}

float ufr_args_getf(const ufr_args_t* args, const char* noun, const float default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_flex_text(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'd' ) {
                    return args->arg[count_arg].i32;
                } else if ( token[1] == 's' ) {
                    return atof(args->arg[count_arg].str);
                } else if ( token[1] == 'f' ) {
                    return args->arg[count_arg].f32;
                }
            } else {
                return atof(token);
            }
        }
    }

    // not found, return default value
    return default_value;
}

const void* ufr_args_getp(const ufr_args_t* args, const char* noun, const void* default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_flex_text(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'p' ) {
                    return args->arg[count_arg].ptr;
                } else {
                    return default_value;
                }
            }
        }
    }

    // not found, return default value
    return default_value;
}

const char* ufr_args_gets(const ufr_args_t* args, const char* noun, const char* default_value) {
    static uint8_t shared_i = 0;
    const uint8_t shared_max = 8;
    static char shared_data[8][128];

    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_flex_text(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 's' ) {
                    return args->arg[count_arg].str;
                } else {
                    return default_value;
                }
            } else {
                char* retval = &shared_data[shared_i][0];
                shared_i = (shared_i+1) % shared_max;
                strcpy(retval, token);
                return retval;
            }
        }
    }

    // not found, return default value
    return default_value;
}

void ufr_put_log(link_t* link, uint8_t level, const char* func_name, const char* format, ...) {
    if ( level > link->log_level ) {
        return;
    }
    va_list list;
    va_start(list, format);
    const int space = 24U - strlen(func_name);
    fprintf(stderr, "# info:%d %d: %s%*s: ", level, link->log_level, func_name, space, "");
    fprintf(stderr, "%*s", link->log_ident, "");
    vfprintf(stderr, format, list);
    fprintf(stderr, "\n");
    va_end(list);
}

int ufr_put_log_error(link_t* link, int error, const char* func_name, const char* format, ...) {
    // copy the error message in the link buffer
    va_list list;
    va_start(list, format);
    vsnprintf(&link->errstr[0], sizeof(link->errstr), format, list);
    va_end(list);

    // show the debug
    if ( link->log_level > 0 ) {
        const int space = 24U - strlen(func_name);
        fprintf(stderr, "\x1B[31m# erro: %s%*s\033[0m: ", func_name, space, "");
        fprintf(stderr, "%s", &link->errstr[0]);
        fprintf(stderr, "\n");
    }

    // return error number
    return error;
}

int  ufr_put_log_error_ident(link_t* link, int error, const char* func_name, const char* format, ...) {
    // copy the error message in the link buffer
    va_list list;
    va_start(list, format);
    vsnprintf(&link->errstr[0], sizeof(link->errstr), format, list);
    va_end(list);

    // show the debug
    if ( link->log_level > 0 ) {
        const int space = 24U - strlen(func_name);
        fprintf(stderr, "\x1B[31m# erro: %s%*s\033[0m: ", func_name, space, "");
        fprintf(stderr, "%s", &link->errstr[0]);
        fprintf(stderr, "\n");
    }

    if ( link->log_ident > 0 ) {
        link->log_ident -= 1;
    }

    // return error number
    return error;
}

void ufr_assert(bool condition, const char* message) {
    exit(1);
}

bool ufr_is_valid(const link_t* link) {
    return link->gtw_api != NULL;
}

bool ufr_is_blank(const link_t* link) {
    return link->gtw_api == NULL;
}

bool ufr_link_is_error(const link_t* link) {
    return link->slot_gtw == 0;
}

const char* ufr_test_args(const link_t* link) {
    return link->gtw_api->test_args(link);
}


size_t  ufr_dummy_read(link_t* link, char* buffer, size_t size) {
    return 0;
}

size_t ufr_dummy_write(link_t* link, const char* buffer, size_t size) {
    return 0;
}

bool ufr_dummy_recv(link_t* link) {
    return false;
}

int ufr_dummy_send(link_t* link) {
    return 0;
}

link_t ufr_accept(link_t* link) {
    link_t client;
    link->gtw_api->accept(link, &client);
    return client;
}

int ufr_recv_peer_name(link_t* link, char* buffer, size_t maxbuffer) {
    return link->gtw_api->recv_peer_name(link, buffer, maxbuffer);
}

// ============================================================================
//  Buffer
// ============================================================================

ufr_buffer_t* ufr_buffer_new() {
    ufr_buffer_t* buffer = malloc(sizeof(ufr_buffer_t));
    if ( buffer != NULL ) {
        ufr_buffer_init(buffer);
    }
    return buffer;
}

void ufr_buffer_init(ufr_buffer_t* buffer) {
    buffer->size = 0;
    buffer->max = MESSAGE_ITEM_SIZE;
    buffer->ptr = malloc(buffer->max);
}

void ufr_buffer_clear(ufr_buffer_t* buffer) {
    buffer->size = 0;
}

void ufr_buffer_free(ufr_buffer_t* buffer) {
    if ( buffer->ptr != NULL ) {
        free(buffer->ptr);
        buffer->ptr = NULL;
    }
    buffer->max = 0;
    buffer->size = 0;
}

static
void ufr_buffer_check_size(ufr_buffer_t* buffer, size_t size) {
    if ( buffer->size + size >= buffer->max ) {
        const size_t new_max = buffer->max * 2;
        char* new_ptr = realloc(buffer->ptr, new_max);
        if ( new_ptr ) {
            buffer->max = new_max;
            buffer->ptr = new_ptr;
        }
    }
}

void ufr_buffer_put(ufr_buffer_t* buffer, char* text, size_t size) {
    ufr_buffer_check_size(buffer, size);
    memcpy(&buffer->ptr[buffer->size], text, size);
    buffer->size += size;
}

void ufr_buffer_put_chr(ufr_buffer_t* buffer, char val) {
    ufr_buffer_check_size(buffer, 1);
    buffer->ptr[buffer->size] = val;
    buffer->size += 1;
}

void ufr_buffer_put_u8_as_str(ufr_buffer_t* buffer, uint8_t val) {
    ufr_buffer_check_size(buffer, 8);
    char* base = &buffer->ptr[buffer->size];
    const size_t size = snprintf(base, 8, "%u ", val);
    buffer->size += size;
}

void ufr_buffer_put_i8_as_str(ufr_buffer_t* buffer, int8_t val) {
    ufr_buffer_check_size(buffer, 8);
    char* base = &buffer->ptr[buffer->size];
    const size_t size = snprintf(base, 8, "%u ", val);
    buffer->size += size;
}

void ufr_buffer_put_u32_as_str(ufr_buffer_t* buffer, uint32_t val) {
    ufr_buffer_check_size(buffer, 32);
    char* base = &buffer->ptr[buffer->size];
    const size_t size = snprintf(base, 32, "%u ", val);
    buffer->size += size;
}

void ufr_buffer_put_i32_as_str(ufr_buffer_t* buffer, int32_t val) {
    ufr_buffer_check_size(buffer, 32);
    char* base = &buffer->ptr[buffer->size];
    const size_t size = snprintf(base, 32, "%d ", val);
    buffer->size += size;
}

void ufr_buffer_put_f32_as_str(ufr_buffer_t* buffer, float val) {
    ufr_buffer_check_size(buffer, 32);
    char* base = &buffer->ptr[buffer->size];
    const size_t size = snprintf(base, 32, "%f ", val);
    buffer->size += size;
}

void ufr_buffer_put_str(ufr_buffer_t* buffer, char* text) {
    const size_t size = strlen(text);
    ufr_buffer_check_size(buffer, size);
    ufr_buffer_put(buffer, text, size);
}

