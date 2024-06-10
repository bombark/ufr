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

uint8_t g_libraries_len = 1;   // 0: error, 1-255 is loaded library
ufr_library_t g_libraries[255];

extern uint8_t g_default_log_level;

// Standard Decoder and Encoder
int ufr_enc_sys_new_std(link_t* link, int type);
int ufr_dcr_sys_new_std(link_t* link, int type);

// ============================================================================
//  Constructor
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
        ufr_start_server(&g_file_out, NULL);
        g_file_in_ptr = &g_file_out;

        // exit
        return;
    }

    // Initialize the g_file_in with UFR_APP_command_STDIN parameters
    snprintf(env_name, sizeof(env_name), "UFR_APP_%s_STDOUT", command);
    const char* link_params = getenv(env_name);
    if ( link_params != NULL ) {
        g_file_out = ufr_new(link_params);
        ufr_start_publisher(&g_file_out, NULL);
    } else {
        g_file_out = ufr_new("@new posix:stdout @encoder std:csv");
    }

    // Initialize the g_file_in with UFR_APP_command_STDIN parameters
    snprintf(env_name, sizeof(env_name), "UFR_APP_%s_STDIN", command);
    const char* link_params_in = getenv(env_name);
    if ( link_params_in != NULL ) {
        g_file_in = ufr_new(link_params_in);
        ufr_start_subscriber(&g_file_in, NULL);
    } else {
        g_file_in = ufr_new("@new posix:stdin @decoder std:csv @sep ,");
    }
}

// ============================================================================
//  ufr_sys_lib_call
// ============================================================================

const char* ufr_sys_lib_call_list (const uint8_t slot, const uint8_t list_idx) {
    // get the function pointer
    char func_name[512];
    ufr_library_t* library = &g_libraries[slot];

    snprintf(func_name, sizeof(func_name), "%s_list", library->name);
    dl_func_list_t dl_func_list = (dl_func_list_t) dlsym(library->handle, func_name);
    if ( dl_func_list == NULL ) {
        return NULL;
    }

    // execute the function
    return dl_func_list(list_idx);
}

int ufr_sys_lib_call_new (
    link_t* link, const uint8_t slot, const char* name, const int type
) {
    if ( name == NULL ) {
        return ufr_error(link, 1, "name is null");
    }

    // get the function pointer
    char func_name[512];
    ufr_library_t* library = &g_libraries[slot];
    if ( name[0] == '\0' ) {
        snprintf(func_name, sizeof(func_name), "%s_new", library->name);
    } else {
        snprintf(func_name, sizeof(func_name), "%s_new_%s", library->name, name);
    }
    
    dl_func_new_t dl_func_new = (dl_func_new_t) dlsym(library->handle, func_name);
    if ( dl_func_new == NULL ) {
        return ufr_error(link, 1, dlerror());
    }

    // execute the function
    ufr_info(link, "calling the function %s(%p)", func_name, dl_func_new);
    return dl_func_new(link, type);
}

// ============================================================================
//  Loading Library System
// ============================================================================

int sys_ufr_load_library (
    link_t* link, const char* lib_file, const char* lib_fullname,
    uint8_t* out_slot
) {
    ufr_info(link, "loading library %s", lib_fullname);

    // open the dinamic library handle
    void* dl_handle = dlopen(lib_file, RTLD_LAZY);
    if ( dl_handle == NULL ) {
        return ufr_error(link, 1, dlerror());
    }

    // find if this handle was already opened
    uint8_t slot = 0;
    bool is_not_loaded = true;
    for (uint8_t i=0; i<g_libraries_len; i++) {
        if ( g_libraries[i].handle == dl_handle ) {
            is_not_loaded = false;
            slot = i;
            break;
        }
    }

    // get the library slot
    ufr_library_t* library;

    // -- create new slot for the library
    if ( is_not_loaded ) {
        slot = g_libraries_len;
        ufr_info(link, "loaded library %s no slot %d", lib_fullname, slot);
        g_libraries_len += 1;
        library = &g_libraries[slot];
        library->count = 0;
        library->handle = dl_handle;
        strcpy(library->name, lib_fullname);

    // -- library is already loaded, get the slot
    } else {
        library = &g_libraries[slot];
    }

    // end
    library->count += 1;
    *out_slot = slot;
    return UFR_OK;
}

int ufr_load_gtw_from_lib(link_t* link, const char* lib_file, const char* lib_name) {
    uint8_t slot;
    const int state = sys_ufr_load_library(link, lib_file, lib_name, &slot);
    if ( state != UFR_OK ) {
        return state;
    }

    link->slot_gtw = slot;
    return UFR_OK;
}

/** load the library, ex: 'gtw' and 'posix' -> libufr_gtw_posix.so
 * @param[in] library_type Type of the library (ex: gtw,enc,dcr)
 * @param[in] library_name nam of library (ex: posix, msgpack, std)
 */
static
int sys_ufr_load (
    link_t* link, const char* library_type, 
    const char* class_path, int boot_type, const ufr_args_t* args
) {
    //
    char lib_name[128];
    char class_name[128];
    uint16_t flex_cursor = 0;
    ufr_flex_text_div(class_path, &flex_cursor, lib_name, sizeof(lib_name), ':');
    ufr_flex_text_div(class_path, &flex_cursor, class_name, sizeof(class_name), ':');

    // load the library
    uint8_t slot;
    char lib_file[512];
    char lib_fullname[256];
    snprintf(lib_fullname, sizeof(lib_fullname), "ufr_%s_%s", library_type, lib_name);
    snprintf(lib_file, sizeof(lib_file), "lib%s.so", lib_fullname);
    const int state1 = sys_ufr_load_library(link, lib_file, lib_fullname, &slot);
    if ( state1 != UFR_OK ) {
        return state1;
    }

    // fill the link->[dcr,enc or gtw]_api
    ufr_sys_lib_call_new(link, slot, class_name, boot_type);

    // store the slot and call the boot
    if ( strcmp(library_type, "dcr") == 0 ) {
        link->slot_dcr = slot;
        ufr_boot_dcr(link, args);
    } else if ( strcmp(library_type, "enc") == 0 ) {
        link->slot_enc = slot;
        ufr_boot_enc(link, args);
    } else if ( strcmp(library_type, "gtw") == 0 ) {
        link->slot_gtw = slot;
        ufr_boot_gtw(link, args);
    } else {
        return ufr_error(link, 1, "invalid boot type");
    }

    // end
    return UFR_OK;
}

/**
 * 
 */
int sys_ufr_new_link(link_t* link, int boot_type, const ufr_args_t* args) {
    ufr_info(link, "creating link");

    // load and boot the gateway
    const char* arg_new = ufr_args_gets(args, "@new", NULL);
    if ( arg_new == NULL ) {
    	link->slot_gtw = 0;
        return ufr_error(link, EINVAL, "Parameter @new is not present");
    }
    sys_ufr_load(link, "gtw", arg_new, boot_type, args);

    // start the gateway, case specified (publisher, subscriver, server or client)
    if ( boot_type != UFR_START_BLANK ) {
        ufr_start(link, boot_type, args);
    }

    // load default encoder or decoder
    const char* coder_name = ufr_args_gets(args, "@coder", NULL);

    // load the encoder, when indicated
    if ( boot_type != UFR_START_SUBSCRIBER ) {
        const char* encoder_name = ufr_args_gets(args, "@encoder", coder_name);
        if ( encoder_name != NULL ) {
            sys_ufr_load(link, "enc", encoder_name, boot_type, args);
        } else {
            ufr_enc_sys_new_std(link, boot_type);
            ufr_boot_enc(link, args);
        }
    }

    // load the decoder, when indicated
    if ( boot_type != UFR_START_PUBLISHER ) {
        const char* decoder_name = ufr_args_gets(args, "@decoder", coder_name);
        if ( decoder_name != NULL ) {
            sys_ufr_load(link, "dcr", decoder_name, boot_type, args);
        } else {
            ufr_dcr_sys_new_std(link, boot_type);
            ufr_boot_dcr(link, args);
        }
    }

    // success
    return UFR_OK;
}

int ufr_link_with_type(link_t* link, const char* text, int boot_type) {
    ufr_init_link(link, NULL);
    link->log_ident = 0;

    if ( text == NULL ) {
        return ufr_error(link, 1, "text parameter is null");
    }
    const ufr_args_t args = {.text=text};
    link->log_level = ufr_args_geti(&args, "@debug", g_default_log_level);
    return sys_ufr_new_link(link, boot_type, &args);
}

link_t ufr_new(const char* text) {
    link_t link;
    ufr_link_with_type(&link, text, 0);
    return link;
}

link_t ufr_link(const char* text) {
    return ufr_new(text);
}

link_t ufr_publisher(const char* text) {
    link_t link;
    ufr_link_with_type(&link, text, UFR_START_PUBLISHER);
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
    link_t link;
    ufr_link_with_type(&link, text, UFR_START_SUBSCRIBER);
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

link_t ufr_server(const char* text) {
    link_t link;
    ufr_link_with_type(&link, text, UFR_START_SERVER);
    return link;
}

link_t ufr_client(const char* text) {
    link_t link;
    ufr_link_with_type(&link, text, UFR_START_CLIENT);
    return link;
}

// ============================================================================
//  Input and Output
// ============================================================================

void ufr_output_init(const char* text) {
    g_file_out = ufr_publisher(text);
}

void ufr_output(const char* format, ...) {
    va_list list;
    va_start(list, format);
    ufr_put_va(&g_file_out, format, list);
    va_end(list);
}

void ufr_output_ln(const char* format, ...) {
    va_list list;
    va_start(list, format);
    ufr_put_va(&g_file_out, format, list);
    va_end(list);
    ufr_put(&g_file_out, "\n");
}

void ufr_input_init(const char* text) {
    g_file_in = ufr_subscriber(text);
}

void ufr_input(const char* format, ...) {
    va_list list;
    va_start(list, format);
    ufr_get_va(g_file_in_ptr, format, list);
    va_end(list);
}

void ufr_inoutput_init(const char* text) {
    g_file_out = ufr_publisher(text);
    ufr_start_server(&g_file_out, NULL);
    g_file_in_ptr = &g_file_out;
}

bool ufr_input_recv() {
    return ufr_recv(g_file_in_ptr);
}

link_t ufr_sys_open(const char* name, const char* def_args) {
    link_t link = {.gtw_api = NULL, .dcr_api = NULL, .enc_api = NULL};
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
