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
#include <string.h>
#include <dlfcn.h>
#include <errno.h>

#include "ufr.h"

link_t g_file_out;
link_t g_file_in;
link_t* g_file_in_ptr = &g_file_in;   // modified when the mode STDINOUT is activated

typedef int (*dl_func_new_t) (link_t*, int type);
typedef const char* (*dl_func_list_t) (const uint8_t idx);


char g_ld_lib_path[1024];



uint8_t g_libraries_len = 0;
ufr_library_t g_libraries[16];

// ============================================================================
//  Functions
// ============================================================================

__attribute__((constructor))
void ufr_sys_init() {
    g_ld_lib_path[0] = '\0';
    return;

    // get the last name of executable (ex. /usr/bin/command -> command) 
    char command[1024];
    command[0] = '\0';
    const char* line = getenv("_");
    if ( line ) {
        const size_t line_len = strlen(line);
        size_t i = line_len;
        for (; i>0; i--) {
            const char c = line[i];
            if ( c == '/' ) {
                i += 1;
                break;
            }
        }
        strcpy(command, &line[i]);
    }

    char env_name[2048];
    snprintf(env_name, sizeof(env_name), "UFR_APP_%s_STDINOUT", command);
    const char* link_inout_params = getenv(env_name);
    if ( link_inout_params != NULL ) {
        g_file_out = ufr_new(link_inout_params);
        ufr_start_bind(&g_file_out, NULL);
        g_file_in_ptr = &g_file_out;

        // exit
        return;
    }

    // Initialize the g_file_in with LT_APP_command_STDIN parameters
    snprintf(env_name, sizeof(env_name), "UFR_APP_%s_STDOUT", command);
    const char* link_params = getenv(env_name);
    if ( link_params != NULL ) {
        g_file_out = ufr_new(link_params);
        ufr_start_publisher(&g_file_out, NULL);
    } else {
        g_file_out = ufr_new("@new posix:stdout @encoder std:csv");
    }

    // Initialize the g_file_in with LT_APP_command_STDIN parameters
    snprintf(env_name, sizeof(env_name), "UFR_APP_%s_STDIN", command);
    const char* link_params_in = getenv(env_name);
    if ( link_params_in != NULL ) {
        g_file_in = ufr_new(link_params_in);
        ufr_start_subscriber(&g_file_in, NULL);
    } else {
        g_file_in = ufr_new("@new posix:stdin @decoder std:csv @sep ,");
    }
}

int sys_ufr_load_gtw_library(link_t* link, const char* name) {
    // open the dinamic library handle
    char filename[512];
    snprintf(filename, sizeof(filename), "lib%s.so", name);
    void* dl_handle = dlopen(filename, RTLD_LAZY);
    if ( dl_handle == NULL ) {
        return lt_error(link, 1, dlerror());
    }

    // find if this handle was already opened
    uint8_t idx = 0;
    bool is_not_loaded = true;
    for (uint8_t i=0; i<g_libraries_len; i++) {
        if ( g_libraries[i].handle == dl_handle ) {
            idx = i;
            break;
        }
    }

    // get the library slot
    ufr_library_t* library;
    if ( is_not_loaded ) {
        idx = g_libraries_len;
        lt_info(link, "loaded library %s no slot %d\n", name, idx);
        g_libraries_len += 1;
        library = &g_libraries[idx];
        library->count = 0;
        library->handle = dl_handle;
        strcpy(library->name, name);
    } else {
        library = &g_libraries[idx];
    }

    // end
    library->count += 1;
    link->library = library;
    return LT_OK;
}

// ============================================================================
//  ufr_sys_lib_call
// ============================================================================

const char* ufr_sys_lib_call_list(link_t* link, uint8_t idx) {
    if ( link->library == NULL ) {
        lt_error(link, 1, "library is not loaded");
        return NULL;
    }

    // get the function pointer
    char func_name[512];
    void* handle = link->library->handle;
    snprintf(func_name, sizeof(func_name), "%s_list", link->library->name);
    dl_func_list_t dl_func_list = (dl_func_list_t) dlsym(handle, func_name);
    if ( dl_func_list == NULL ) {
        lt_error(link, 1, dlerror());
        return NULL;
    }

    // 
    return dl_func_list(idx);
}

int ufr_sys_lib_call_new(link_t* link, const char* name, int type) {
    if ( link->library == NULL ) {
        return lt_error(link, 1, "library is not loaded");
    }

    // get the function pointer
    char func_name[512];
    void* handle = link->library->handle;
    snprintf(func_name, sizeof(func_name), "%s_new_%s", link->library->name, name);
    dl_func_new_t dl_func_new = (dl_func_new_t) dlsym(handle, func_name);
    if ( dl_func_new == NULL ) {
        return lt_error(link, 1, dlerror());
    }

    // 
    return dl_func_new(link, type);
}

// ============================================================================
//  Loading Library System
// ============================================================================

/** load the library, ex: 'gtw' and 'posix' -> libufr_gtw_posix.so
 * @param[in] library_type Type of the library (ex: gtw,ecr,dcr)
 * @param[in] library_name nam of library (ex: posix, msgpack, std)
 */
static
int sys_ufr_load(link_t* link, const char* library_type, const char* library_name) {
    char lib_fullname[256];
    snprintf(lib_fullname, sizeof(lib_fullname), "ufr_%s_%s", library_type, library_name);
    return sys_ufr_load_gtw_library(link, lib_fullname);
}


int sys_ufr_new_link(link_t* link, int type, const lt_args_t* args) {
    lt_info(link, "creating link");

    // get argument @new
    const char* arg_new = lt_args_gets(&args, "@new", NULL);
    if ( arg_new == NULL ) {
        return lt_error(link, EINVAL, "Parameter @new is not present");
    }

    // split the parameter @new to library_name and class_name
    char library_name[128];
    char class_name[128];
    uint16_t class_name_cur = 0;
    lt_flex_text_div(arg_new, &class_name_cur, library_name, sizeof(library_name), ':');
    lt_flex_text_div(arg_new, &class_name_cur, class_name, sizeof(class_name), ':');

    // load the gateway
    lt_info(link, "- loading gateway %s loaded with (%s)", arg_new, args.text);
    const int error1 = sys_ufr_load_gtw_library(link, class_name, args);
    if ( error1 != LT_OK ) {
        return error1;
    }

    // fill the link->gtw_api
    ufr_sys_lib_call_new(link, class_name, args);
    ufr_boot(link, args);

    lt_info(link, "- gateway %s loaded", class_name);

    // load default encoder or decoder
    const char* coder_name = lt_args_gets(&args, "@coder", NULL);

    // load the encoder, when indicated
    const char* encoder_name = lt_args_gets(&args, "@encoder", coder_name);
    if ( encoder_name != NULL ) {
        lt_info(link, "- loading encoder %s", encoder_name);
        const int error2 = lt_load_encoder(link, encoder_name, &args);
        if ( error2 != LT_OK ) {
            return error2;
        }
        lt_info(link, "- encoder %s loaded", encoder_name);
    }

    // load the decoder, when indicated
    const char* decoder_name = lt_args_gets(&args, "@decoder", coder_name);
    if ( decoder_name != NULL ) {
        lt_info(link, "- loading decoder %s", decoder_name);
        const int error3 = lt_load_decoder(link, decoder_name, &args);
        if ( error3 != LT_OK ) {
            return error3;
        }
        lt_info(link, "- decoder %s loaded", decoder_name);
    }

    // success
    return LT_OK;
}









int lt_load_gateway(link_t* link, const char* class_name, const lt_args_t* args) {

    const int state1 = lt_load_lib(link, "gtw", class_name, args);
    if ( state1 != LT_OK ) {
        return state1;
    }

    return ufr_boot(link, link->type_started);
}

int lt_load_encoder(link_t* link, const char* class_name, const lt_args_t* args) {
    return lt_load_lib(link, "ecr", class_name, args);
}

int lt_load_decoder(link_t* link, const char* class_name, const lt_args_t* args) {
    return lt_load_lib(link, "dcr", class_name, args);
}



link_t ufr_link(const char* text) {
    link_t link = {.gw_api=NULL, .dec_api=NULL, .enc_api=NULL};
    if ( text != NULL ) {
        lt_new_ptr(&link, text);
    }
    return link;
}

link_t ufr_publisher(const char* text) {
    link_t link = {.gw_api=NULL, .dec_api=NULL, .enc_api=NULL};
    if ( text == NULL ) {
        return link;
    }

    



    return link;
}

link_t ufr_sys_publisher(const char* name, const char* default_text) {
    char var_name[512];
    snprintf(var_name, sizeof(var_name), "UFR_SYS_%s", name);
    char const* text = getenv(var_name);
    if ( text == NULL ) {
        text = default_text;
    }
    return ufr_publisher(text);
}

link_t ufr_subscriber(const char* text) {
    link_t link = {.gw_api=NULL, .dec_api=NULL, .enc_api=NULL};
    link.type_started = LT_START_SUBSCRIBER;
    if ( text != NULL ) {
        if ( lt_new_ptr(&link, text) == LT_OK ) {
            lt_args_t sub_args = {.text=text};
            ufr_start_subscriber(&link, &sub_args);
        }
    }
    return link;
}

link_t ufr_sys_subscriber(const char* name, const char* default_text) {
    char var_name[512];
    snprintf(var_name, sizeof(var_name), "UFR_SYS_%s", name);
    char const* text = getenv(var_name);
    if ( text == NULL ) {
        text = default_text;
    }
    return ufr_subscriber(text);
}

void ufr_output_init(const char* text) {
    g_file_out = ufr_publisher(text);
}

void ufr_output(const char* format, ...) {
    va_list list;
    va_start(list, format);
    lt_put_va(&g_file_out, format, list);
    va_end(list);
}

void ufr_output_ln(const char* format, ...) {
    va_list list;
    va_start(list, format);
    lt_put_va(&g_file_out, format, list);
    va_end(list);
    lt_put(&g_file_out, "\n");
}

void ufr_input_init(const char* text) {
    g_file_in = ufr_subscriber(text);
}


void ufr_input(const char* format, ...) {
    va_list list;
    va_start(list, format);
    lt_get_va(g_file_in_ptr, format, list);
    va_end(list);
}

void ufr_inoutput_init(const char* text) {
    g_file_out = ufr_publisher(text);
    ufr_start_bind(&g_file_out, NULL);
    g_file_in_ptr = &g_file_out;
}

bool ufr_input_recv() {
    return lt_recv(g_file_in_ptr);
}

link_t ufr_sys_open(const char* name, const char* def_args) {
    link_t link = {.gw_api = NULL, .dec_api = NULL, .enc_api = NULL};
    char env_name[512];
    snprintf(env_name, sizeof(env_name), "UFR_SYS_%s", name);
    const char* link_params = getenv(env_name);
    if ( link_params != NULL ) {
        link = ufr_new(link_params);
    } else {
        link = ufr_new(def_args);
    }
    return link; 
}

void urf_sys_set_ld_path(char* path) {
    size_t len = strlen(path);
    strcpy(g_ld_lib_path, path);
    if ( len > 0 ) {
        if ( g_ld_lib_path[len-1] != '/' ) {
            g_ld_lib_path[len] = '/';
            g_ld_lib_path[len+1] = '\0';
        }
    }
}