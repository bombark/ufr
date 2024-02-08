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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>

#define LT_OK  0

#define LINK_TO_ERROR  0x00

#define LT_SIZE_STD        0
#define LT_SIZE_MAX        1

#define LT_START_BLANK      0
#define LT_START_CONNECT    1
#define LT_START_BIND       2
#define LT_START_PUBLISHER  3
#define LT_START_SUBSCRIBER 4

#define LT_STOP_CLOSE       1



#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
//  Parameters
// ============================================================================

// 8 bytes
typedef union {
	uint32_t    u32;
	uint64_t    u64;
	int32_t     i32;
	int32_t     i64;
	float       f32;
	double      f64;
	void*       ptr;
	char const* str;
} item_t;

// 64 bytes for #64
typedef struct {
    const char* text;
    item_t arg[7];
} lt_args_t;

// ============================================================================
//  API
// ============================================================================

struct _link;

typedef struct {
	const char* name;

	// certo
	int    (*type)(const struct _link* link);
	int    (*state)(const struct _link* link);
	size_t (*size)(const struct _link* link, int type);

	// certo
	int  (*boot)(struct _link* link, const lt_args_t* args);
	int  (*start)(struct _link* link, int type, const lt_args_t* args);
	void (*stop)(struct _link* link, int type);
	int  (*copy)(struct _link* link, struct _link* out);

	// certo
	size_t (*read)(struct _link* link, char* buffer, size_t length);
	size_t (*write)(struct _link* link, const char* buffer, size_t length);

	bool (*recv)(struct _link* link);
	bool (*recv_async)(struct _link* link);

    int (*send)(struct _link* link);
    int (*accept)(struct _link* link, struct _link* out_client);
} lt_api_t;

typedef struct {
    int (*boot)(struct _link* link, const lt_args_t* args);
    void (*close)(struct _link* link);

	void (*recv)(struct _link* link, char* msg_data, size_t msg_size);

	int (*get_u32)(struct _link* link, uint32_t* val);
	int (*get_i32)(struct _link* link, int32_t* val);
	int (*get_f32)(struct _link* link, float* ret_val);
	int (*get_str)(struct _link* link, char** ret_val);
    
	int (*get_arr)(struct _link* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr);

    int (*copy_str)(struct _link* link, char* ret_val, size_t size_max);

	int (*copy_arr)(struct _link* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr);
} lt_decoder_api_t;

typedef struct {
    int (*boot)(struct _link* link, const lt_args_t* args);
    void (*close)(struct _link* link);
    void (*clear)(struct _link* link);
    int (*set_header)(struct _link* link, const char* header);

	int (*put_u32)(struct _link* link, uint32_t val);
	int (*put_i32)(struct _link* link, int32_t val);
	int (*put_f32)(struct _link* link, float val);
	int (*put_str)(struct _link* link, const char* val);
	int (*put_cmd)(struct _link* link, char cmd);

	int (*put_arr)(struct _link* link, const void* array, char type, size_t size);
	int (*put_mat)(struct _link* link, const void* vet, char type, size_t rows, size_t cols);

    int (*enter_array)(struct _link* link, size_t maxsize);
    int (*leave_array)(struct _link* link);

} lt_encoder_api_t;

// ============================================================================
//  Link
// ============================================================================

typedef struct {
    void* handle;
    size_t count;
    char name[240];
} ufr_library_t;

typedef struct _link {
    // Gateway
    union {
	    lt_api_t* gw_api;
        const lt_api_t* gtw_api;
    };

    union {
        void*     gw_shr;
        void*     gtw_shr;
    };

    union {
        void*     gw_obj;
        void*     gtw_obj;
    };

    // Encoder
    union {
        lt_encoder_api_t* enc_api;
        lt_encoder_api_t* ecr_api;
    };

    union {
	    void*             enc_obj;
        void*             ecr_obj;
    };

    // Decoder
    union {
        lt_decoder_api_t* dec_api;
        lt_decoder_api_t* dcr_api;
    };

    union {
        void*             dec_obj;
        void*             dcr_obj;
    };

    uint8_t type_started;
    uint8_t log_level;
    char errstr[126];

    void* dl_handle;

    ufr_library_t* library;
    
    uint8_t slot_lib_gtw;
    uint8_t slot_lib_ecr;
    uint8_t slot_lib_dcr;
} link_t;


// ============================================================================
//  Link Functions
// ============================================================================

// Meta data
const char* lt_api_name(const link_t* link);
int lt_type(const link_t* link);
int lt_state(const link_t* link);
size_t lt_size(const link_t* link);
size_t lt_size_max(const link_t* link);

int ufr_state(const link_t* link);
bool ufr_link_is_pusblisher(const link_t* link);
bool ufr_link_is_subscriber(const link_t* link);
bool ufr_link_is_server(const link_t* link);
bool ufr_link_is_client(const link_t* link);

// Start and stop
void lt_init_api(link_t* link, lt_api_t* gw_api);
// int  lt_boot(link_t* link, const lt_args_t* args);
// int  lt_boot_va(link_t* link, const char* text, ...);

// system new
int ufr_init(link_t* link, const char* package_name, const char* class_name);
link_t ufr_new(const char* text);
link_t ufr_publisher(const char* text);
link_t ufr_subscriber(const char* text);
link_t ufr_server(const char* text);
link_t ufr_client(const char* text);

// boot
int ufr_boot_dcr(link_t* link, const lt_args_t* args);
int ufr_boot_ecr(link_t* link, const lt_args_t* args);
int ufr_boot_gtw(link_t* link, const lt_args_t* args);


// start
int ufr_start(link_t* link, const lt_args_t* param_args);
int ufr_start_publisher(link_t* link, const lt_args_t* args);
int ufr_start_subscriber(link_t* link, const lt_args_t* args);
int ufr_start_bind(link_t* link, const lt_args_t* args);
int ufr_start_connect(link_t* link, const lt_args_t* args);

// stop
void lt_stop(link_t* link);
void lt_close(link_t* link);
inline void ufr_close(link_t* link){ lt_close(link); }

bool lt_recv(link_t* link);
bool lt_recv_async(link_t* link);
bool ufr_send(link_t* link);

size_t lt_read(link_t* node, char* buffer, size_t size);
size_t lt_write(link_t* node, const char* buffer, size_t size);

void lt_get_va(link_t* link, const char* format, va_list list);
void lt_get(link_t* link, char* format, ...);

bool ufr_get_str(link_t* link, char* buffer);

void lt_put_va(link_t* link, const char* format, va_list list);
void lt_put(link_t* link, const char* format, ...);

size_t lt_copy_ai32(link_t* link, size_t arr_size_max, int32_t* arr_data);
size_t lt_copy_af32(link_t* link, size_t arr_size_max, float* arr_data);

int ufr_enter_array(link_t* link, size_t arr_size_max);
int ufr_leave_array(link_t* link);

// Functions on lt_args_t
bool lt_flex_text_div(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max, const char div);

size_t lt_args_getu(const lt_args_t* args, const char* noun, const size_t default_value);
int    lt_args_geti(const lt_args_t* args, const char* noun, const int default_value);
const void* lt_args_getp(const lt_args_t* args, const char* noun, const void* default_value);
const char* lt_args_gets(const lt_args_t* args, const char* noun, const char* default_value);


bool lt_is_valid(const link_t* link);
bool lt_is_blank(const link_t* link);

// ============================================================================
//  Dummy functions
// ============================================================================

size_t ufr_dummy_read (link_t*, char*, size_t);
size_t ufr_dummy_write(link_t*, const char*, size_t);
bool   ufr_dummy_recv (link_t*);
int    ufr_dummy_send (link_t*);


// ============================================================================
//  UFR functions
// ============================================================================

// lt_sys.h
int lt_load_gateway(link_t* link, const char* class_name, const lt_args_t* args);
int lt_load_encoder(link_t* link, const char* class_name, const lt_args_t* args);
int lt_load_decoder(link_t* link, const char* class_name, const lt_args_t* args);
int lt_new_ptr(link_t* link, const char* text);

link_t ufr_sys_publisher(const char* var_name, const char* text);
link_t ufr_sys_subscriber(const char* name, const char* default_text);

void ufr_output_init(const char* text);
void ufr_output(const char* format, ...);

void ufr_input_init(const char* text);
void ufr_input(const char* format, ...);
bool ufr_input_recv();

void ufr_inoutput_init(const char* text);

void lt_log(link_t* link, uint8_t level, const char* format, ...);
void lt_log_info(link_t* link, uint8_t level, const char* func_name, const char* format, ...);
int  lt_log_error(link_t* link, int error, const char* func_name, const char* format, ...);


#define lt_warn(link, ...) lt_log_info(link, 1, __func__, __VA_ARGS__)
#define lt_info(link, ...) lt_log_info(link, 2, __func__, __VA_ARGS__)
#define lt_error(link, error, ...) lt_log_error(link, error, __func__, __VA_ARGS__)


link_t ufr_sys_open(const char* name, const char* def_args);


// int ufr_sys_load_library(link_t* link, const char* name);
// const char* ufr_sys_lib_call_list(link_t* link, uint8_t idx);
// int ufr_sys_lib_call_new(link_t* link, const char* name, int type);

void urf_sys_set_ld_path(char* path);

#ifdef __cplusplus
}
#endif