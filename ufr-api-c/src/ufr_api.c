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

uint8_t g_default_log_level = 10;

// ============================================================================
//  Link - Meta
// ============================================================================

const char* lt_api_name(const link_t* link) {
	if ( link->gw_api == NULL ) {
		return "None";
	}
	return link->gw_api->name;
}

int lt_type(const link_t* unit) {
	if ( unit == NULL || unit->gw_api == NULL || unit->gw_api->type == NULL) {
		return LINK_TO_ERROR;
	}
	return unit->gw_api->type(unit);
}

int lt_state(const link_t* unit) {
	if ( unit == NULL || unit->gw_api == NULL || unit->gw_api->state == NULL) {
		return 0;
	}
	return unit->gw_api->state(unit);
}

int ufr_state(const link_t* link) {
    return link->type_started;
}

bool ufr_link_is_publisher(const link_t* link) {
    return link->type_started == LT_START_PUBLISHER;
}

bool ufr_link_is_subscriber(const link_t* link) {
    return link->type_started == LT_START_SUBSCRIBER;
}

bool ufr_link_is_server(const link_t* link) {
    return link->type_started == LT_START_BIND;
}

bool ufr_link_is_client(const link_t* link) {
    return link->type_started == LT_START_CONNECT;
}

size_t lt_size(const link_t* link) {
	if ( link == NULL || link->gw_api == NULL || link->gw_api->size == NULL) {
		return 0;
	}
	return link->gw_api->size(link, LT_SIZE_STD);
}

size_t lt_size_max(const link_t* link) {
	if ( link == NULL || link->gw_api == NULL || link->gw_api->size == NULL) {
		return 0;
	}

	return link->gw_api->size(link, LT_SIZE_MAX);
}

void lt_init_api(link_t* link, lt_api_t* gw_api) {
    link->gw_api = gw_api;
    link->gw_obj = NULL;
    link->gw_shr = NULL;
    link->dec_api = NULL;
    link->dec_obj = NULL;
    link->enc_api = NULL;
    link->enc_obj = NULL;
    link->type_started = LT_START_BLANK;
    link->log_level = g_default_log_level;
}

int ufr_boot_dcr(link_t* link, const lt_args_t* args) {
    lt_info(link, "booting decoder");
    const int state = link->dec_api->boot(link, args);
    return state;
}

int ufr_boot_ecr(link_t* link, const lt_args_t* args) {
    lt_info(link, "booting encoder");
    const int state = link->ecr_api->boot(link, args);
    return state;
}

int ufr_boot_gtw(link_t* link, const lt_args_t* args) {
    lt_info(link, "booting gateway");
    const int state = link->gtw_api->boot(link, args);
    return state;
}

int ufr_start(link_t* link, const lt_args_t* param_args) {
    // select the arguments avoiding NULL pointer
    const lt_args_t empty_args = {.text=""};
    const lt_args_t* args = ( param_args != NULL ) ? param_args : &empty_args;

    // update the debug level
    link->log_level = lt_args_geti(args, "@debug", g_default_log_level);

    // call driver function
    const int type = link->type_started;
    const int error = link->gtw_api->start(link, type, args);

    // end
    return error;
}

int ufr_start_publisher(link_t* link, const lt_args_t* args) {
    link->type_started = LT_START_PUBLISHER;
    return ufr_start(link, args);
}

int ufr_start_subscriber(link_t* link, const lt_args_t* args) {
    link->type_started = LT_START_SUBSCRIBER;
    return ufr_start(link, args);
}

int ufr_start_bind(link_t* link, const lt_args_t* args) {
    link->type_started = LT_START_BIND;
    return ufr_start(link, args);
}

int ufr_start_connect(link_t* link, const lt_args_t* args) {
    link->type_started = LT_START_CONNECT;
    return ufr_start(link, args);
}

bool ufr_recv(link_t* link) {
    if ( link->gw_api == NULL ) {
        return false;
    }
    return link->gw_api->recv(link);
}

bool lt_recv(link_t* link) {
    return ufr_recv(link);
}

bool lt_recv_async(link_t* link) {
    if ( link->gw_api == NULL ) {
        return false;
    }
    return link->gw_api->recv_async(link);
}

bool ufr_send(link_t* link) {
    int error = link->gw_api->send(link);
    return error == LT_OK;
}

void ufr_close(link_t* link) {
    link->gw_api->stop(link, LT_STOP_CLOSE);
}

void lt_close(link_t* link) {
    ufr_close(link);
}

// ============================================================================
//  Link - Character Stream
// ============================================================================

size_t ufr_read(link_t* node, char* buffer, size_t maxsize) {
	if ( node == NULL || node->gw_api == NULL || node->gw_api->read == NULL){
		return 0;
	}

	// fprintf(stderr, "%s::%s %p\n", node->gw_api->name, __func__, node);
	return node->gw_api->read(node, buffer, maxsize);
}

size_t lt_read(link_t* node, char* buffer, size_t maxsize) {
    return ufr_read(node, buffer, maxsize);
}

size_t ufr_write(link_t* node, const char* buffer, size_t size) {
	if ( node == NULL || node->gw_api == NULL || node->gw_api->write == NULL) {
		return 0;
	}

	// fprintf(stderr, "%s::%s %p\n", node->gw_api->name, __func__, node);
	return node->gw_api->write(node, buffer, size);
}

size_t lt_write(link_t* node, const char* buffer, size_t size) {
    return ufr_write(node, buffer, size);
}

// ============================================================================
//  Link - Formatted Stream
// ============================================================================

void lt_get_va(link_t* link, const char* format, va_list list) {
    if ( link->dec_api == NULL ) {
        lt_error(link, 0, "Decoder is not loaded");
        return;
    }

	while( format != NULL ) {	
		const char type = *format;
        format += 1;

		if ( type == '\0' ) {
			break;

		} else if ( type == '^' ) {
			lt_recv(link);

        } else if ( type == 'a' ) {
            const char arr_type = *format;
            format += 1;
            if ( arr_type == '\0' ) {
                break;
            }
            const int32_t arr_size_max = va_arg(list, int32_t);
            // size_t* arr_size_ptr = va_arg(list, size_t*);
            size_t arr_size;
            void* arr_ptr = va_arg(list, void*);
            link->dec_api->copy_arr(link, arr_type, arr_size_max, &arr_size, arr_ptr);

		} else {
            switch (type) {
                case 's': {
                    char* buffer = va_arg(list, char*);
                    link->dec_api->copy_str(link, buffer, -1);
                } break;

                case 'i': {
                    int32_t *val = va_arg(list, int32_t*);
                    link->dec_api->get_i32(link, val);
                } break;

                case 'f': {
                    float* val = va_arg(list, float*);
                    link->dec_api->get_f32(link, val);
                } break;

                case '\n': {
                    // lt_end(link);
                } break;
            }
        }
	}
	
}

void lt_get(link_t* link, char* format, ...) {
    va_list list;
	va_start(list, format);
    lt_get_va(link, format, list);
    va_end(list);
}

bool ufr_get_str(link_t* link, char* buffer) {
    const int is_ok = link->dec_api->copy_str(link, buffer, -1);
    return is_ok == LT_OK;
}

void ufr_put_va(link_t* link, const char* format, va_list list) {
    if ( link->enc_api == NULL ) {
        lt_error(link, 0, "Encoder is not loaded");
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
			link->enc_api->put_cmd(link, '\n');

		} else if ( type == 'a' ) {
            const char arr_type = *format;
            format += 1;
            if ( arr_type == '\0' ) {
                break;
            }
            const int32_t arr_size = va_arg(list, int32_t);
            const void* arr_ptr = va_arg(list, void*);
            link->enc_api->put_arr(link, arr_ptr, arr_type, arr_size);

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
					const double val = va_arg(list, double);
					link->enc_api->put_f32(link, val);
				} break;

				default:
					break;
			}
		}
	}
	
}

void ufr_put(link_t* link, const char* format, ...) {
    va_list list;
    va_start(list, format);
    ufr_put_va(link, format, list);
    va_end(list);
}

void lt_put(link_t* link, const char* format, ...) {
    va_list list;
    va_start(list, format);
    ufr_put_va(link, format, list);
    va_end(list);
}

size_t lt_copy_ai32(link_t* link, size_t arr_size_max, int32_t* arr_data) {
    size_t arr_size = 0;
    link->dec_api->copy_arr(link, 'i', arr_size_max, &arr_size, (void*) arr_data);
    return arr_size;
}

size_t lt_copy_af32(link_t* link, size_t arr_size_max, float* arr_data) {
    size_t arr_size = 0;
    link->dec_api->copy_arr(link, 'f', arr_size_max, &arr_size, (void*) arr_data);
    return arr_size;
}

int ufr_enter_array(link_t* link, size_t arr_size_max) {
    return link->ecr_api->enter_array(link, arr_size_max);
}

int ufr_leave_array(link_t* link) {
    return link->ecr_api->leave_array(link);
}

// ============================================================================
//  Arguments
// ============================================================================

bool lt_flex_text_div(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max, const char div) {
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

static
bool lt_flex_text(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max) {
    return lt_flex_text_div(text, cursor_ini, token, token_max, ' ');
}

size_t lt_args_getu(const lt_args_t* args, const char* noun, const size_t default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( lt_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            lt_flex_text(args->text, &cursor, token, sizeof(token));
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

int lt_args_geti(const lt_args_t* args, const char* noun, const int default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( lt_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            lt_flex_text(args->text, &cursor, token, sizeof(token));
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

const void* lt_args_getp(const lt_args_t* args, const char* noun, const void* default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( lt_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            lt_flex_text(args->text, &cursor, token, sizeof(token));
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

const char* lt_args_gets(const lt_args_t* args, const char* noun, const char* default_value) {
    static uint8_t shared_i = 0;
    const uint8_t shared_max = 8;
    static char shared_data[8][128];

    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( lt_flex_text(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            lt_flex_text(args->text, &cursor, token, sizeof(token));
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

void lt_log(link_t* link, uint8_t level, const char* format, ...) { 
    // if ( link->level >= level ) {
        va_list list;
        va_start(list, format);
        vfprintf(stderr, format, list);
        fprintf(stderr, "\n");
        va_end(list);
    // }
}

void lt_log_info(link_t* link, uint8_t level, const char* func_name, const char* format, ...) {
    /*if ( level > link->log_level ) {
        return;
    }*/
    va_list list;
    va_start(list, format);
    const int space = 24U - strlen(func_name);
    fprintf(stderr, "# info: %s%*s: ", func_name, space, "");
    vfprintf(stderr, format, list);
    fprintf(stderr, "\n");
    va_end(list);
}

int lt_log_error(link_t* link, int error, const char* func_name, const char* format, ...) {
    va_list list;
    va_start(list, format);
    const int space = 24U - strlen(func_name);
    fprintf(stderr, "\x1B[31m# erro: %s%*s\033[0m: ", func_name, space, "");
    vfprintf(stderr, format, list);
    fprintf(stderr, "\n");
    va_end(list);
    return error;
}

void lt_assert(bool condition, const char* message) {
    exit(1);
}

void lt_funclog_begin(link_t* link) {

}

void lt_funclog_end(link_t* link) {
}

bool lt_is_valid(const link_t* link) {
    return link->gw_api != NULL;
}

bool lt_is_blank(const link_t* link) {
    return link->gw_api == NULL;
}




size_t  ufr_dummy_read(link_t*, char*, size_t) {
    return 0;
}

size_t ufr_dummy_write(link_t*, const char*, size_t) {
    return 0;
}

bool ufr_dummy_recv(link_t* link) {
    return false;
}

int ufr_dummy_send(link_t* link) {
    return 0;
}
