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


#define UFR_OK  0

#define LINK_TO_ERROR  0x00

#define UFR_SIZE_STD        0
#define UFR_SIZE_MAX        1

#define UFR_START_BLANK      0
#define UFR_START_SERVER     1
#define UFR_START_SERVER_ST  1
#define UFR_START_SERVER_MT  2
#define UFR_START_CLIENT     3
#define UFR_START_PUBLISHER  4
#define UFR_START_SUBSCRIBER 5

#define UFR_STOP_CLOSE       1

#define UFR_TYPE_SOCKET 1
#define UFR_TYPE_TOPIC  2


#define UFR_STATUS_RESET    0
#define UFR_STATUS_BOOTED   1
#define UFR_STATUS_STARTED  2


#define UFR_STATE_READY     0
#define UFR_STATE_RECV      1
#define UFR_STATE_SEND      2

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
} ufr_args_t;

// ============================================================================
//  API
// ============================================================================

struct _link;

typedef struct {
    int flag;
    size_t size;
    int32_t* data;
} ufr_ai32_t;

typedef struct {
	const char* name;

	// incerto
	int    (*type)(const struct _link* link);  // remover
	int    (*state)(const struct _link* link); // remover
	size_t (*size)(const struct _link* link, int type); // remover

	// certo
	int  (*boot)(struct _link* link, const ufr_args_t* args);
	int  (*start)(struct _link* link, int type, const ufr_args_t* args);
	void (*stop)(struct _link* link, int type);
	int  (*copy)(struct _link* link, struct _link* out);

	// certo
	size_t (*read)(struct _link* link, char* buffer, size_t length);
	size_t (*write)(struct _link* link, const char* buffer, size_t length);

    // receive functions
	int (*recv)(struct _link* link);
	int (*recv_async)(struct _link* link);
    int (*recv_peer_name)(struct _link* link, char* buffer, size_t maxbuffer);

    // server multi-thread
    int (*accept)(struct _link* link, struct _link* out_client);

    // tests
    const char* (*test_args)(const struct _link* link);
} ufr_gtw_api_t;

typedef struct {
    // Open and close
    int  (*boot)(struct _link* link, const ufr_args_t* args);
    void (*close)(struct _link* link);

    // receive callback
	int (*recv_cb)(struct _link* link, char* msg_data, size_t msg_size);
    int (*recv_async_cb)(struct _link* link, char* msg_data, size_t msg_size);

    // Next item
    int (*next)(struct _link* link);

    // Function on current Item
    char     (*get_type)(struct _link* link);
    int      (*get_nbytes)(struct _link* link);
    int      (*get_nitems)(struct _link* link);
    uint8_t* (*get_raw_ptr)(struct _link* link);

    int (*get_raw)(struct _link* link, uint8_t* out, int max_nbytes);
    int (*get_str)(struct _link* link, char* out, int max_nbytes);

	int (*get_u32)(struct _link* link, uint32_t* out, int max_nitems);
	int (*get_i32)(struct _link* link, int32_t* out, int max_nitems);
	int (*get_f32)(struct _link* link, float* out, int max_nitems);

    int (*get_u64)(struct _link* link, uint64_t* out, int max_nitems);
	int (*get_i64)(struct _link* link, int64_t* out, int max_nitems);
	int (*get_f64)(struct _link* link, double* out, int max_nitems);

    int (*enter)(struct _link* link);
    int (*leave)(struct _link* link);
} ufr_dcr_api_t;

typedef struct {
    int  (*boot)(struct _link* link, const ufr_args_t* args);
    void (*close)(struct _link* link);
    void (*clear)(struct _link* link);

    // 32 bits
	int (*put_u32)(struct _link* link, const uint32_t* val, int nitems);
	int (*put_i32)(struct _link* link, const int32_t* val, int nitems);
	int (*put_f32)(struct _link* link, const float* val, int nitems);

    // 64 bits
    int (*put_u64)(struct _link* link, const uint64_t* val, int nitems);
	int (*put_i64)(struct _link* link, const int64_t* val, int nitems);
	int (*put_f64)(struct _link* link, const double* val, int nitems);

    // Single - 8 bits
    int (*put_cmd)(struct _link* link, char cmd);
    int (*put_str)(struct _link* link, const char* val);
    int (*put_raw)(struct _link* link, const uint8_t* val, int nbytes);

    // acho que tah bom, talvez colocar o tipo
    int (*enter)(struct _link* link, size_t max_nitems);
    int (*leave)(struct _link* link);
} ufr_enc_api_t;

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
    const ufr_gtw_api_t* gtw_api;
    void* gtw_shr;
    void* gtw_obj;

    // Encoder
    const ufr_enc_api_t* enc_api;
    void* enc_obj;
    
    // Decoder
    const ufr_dcr_api_t* dcr_api;
    void* dcr_obj;

    // Decoder Stack
    const ufr_dcr_api_t* dcr_api_s0;
    void* dcr1_obj_s0;


    uint8_t type_started;
    uint8_t log_level;
    uint8_t log_ident;
    uint8_t status;

    union {
        struct{
            uint8_t is_booted  :1;
            uint8_t is_started :1;
            uint8_t state      :2;
        };
    };

    uint16_t put_count;

    uint8_t slot_gtw;
    uint8_t slot_enc;
    uint8_t slot_dcr;
    char errstr[180];
} link_t;

typedef struct {
    char const* name;
    ufr_args_t args;
} ufr_node_t;

// ============================================================================
//  Link Functions
// ============================================================================

// Meta data
const char* ufr_api_name(const link_t* link);
// int ufr_gtw_type(const link_t* link);
// int ufr_gtw_state(const link_t* link);
// size_t ufr_size(const link_t* link);
// size_t ufr_size_max(const link_t* link);

bool ufr_link_is_publisher(const link_t* link);
bool ufr_link_is_subscriber(const link_t* link);
bool ufr_link_is_server(const link_t* link);
bool ufr_link_is_client(const link_t* link);

bool ufr_is_valid(const link_t* link);
bool ufr_is_blank(const link_t* link);
bool ufr_link_is_error(const link_t* link);


// Set zeros to link
void ufr_init_link(link_t* link, ufr_gtw_api_t* gtw_api);

// system new
int ufr_init(link_t* link, const char* package_name, const char* class_name);

/**
 * @brief Pensar em remover
 * 
 * @param text 
 * @return link_t 
 */
link_t ufr_new(const char* text);

/**
 * @brief Create a new publisher
 * 
 * @param text parameters for the publisher. Example "@new zmq:topic @coder msgpack"
 * @return link_t opened link
 */
link_t ufr_publisher(const char* text);

/**
 * @brief Create a new subscriber 
 * 
 * @param text 
 * @return link_t 
 */
link_t ufr_subscriber(const char* text);

/**
 * @brief Create a new single thread server
 * 
 * @param text 
 * @return link_t 
 */
link_t ufr_server(const char* text);

/**
 * @brief Create a new single thread server
 * 
 * @param text 
 * @return link_t 
 */
link_t ufr_server_st(const char* text);

/**
 * @brief Create a link as client
 *
 * This function put formatted data to the message and it will be send
 * to the link related. 
 *
 * @param text text with the arguments. Example: aaa 
 * @return created link as client
 * 
 * @version 1.0
 */
link_t ufr_client(const char* text);

// boot
int ufr_boot_dcr(link_t* link, const ufr_args_t* args);
int ufr_boot_enc(link_t* link, const ufr_args_t* args);
int ufr_boot_gtw(link_t* link, const ufr_args_t* args);

int ufr_boot_publisher(link_t* link, const char* text);
int ufr_boot_subscriber(link_t* link, const char* text);

// ============================================================================
//  Start
// ============================================================================

// Eliminar essas funcoes

/*
 * @brief 
 * 
 * @param link link to be started 
 * @param type type to start (publisher, subscriber, client or server)
 * @param param_args variable arguments for start function 
 * @return int returns error code and 0 for success;
 */
int ufr_start(link_t* link, int type, const ufr_args_t* param_args);

/**
 * @brief start a link as topic publisher
 * 
 * @param link link to be started 
 * @param args variable arguments for start function 
 * @return int error code or 0 for success 
 */
int ufr_start_publisher(link_t* link, const ufr_args_t* args);

/**
 * @brief start a link as topic subscriber
 * 
 * @param link link to be started
 * @param args variable arguments for start function
 * @return int error code or 0 for success 
 */
int ufr_start_subscriber(link_t* link, const ufr_args_t* args);

/**
 * @brief start a link as socket server
 * 
 * @param link link to be started 
 * @param args variable arguments for start function 
 * @return int error code or 0 for success 
 */
int ufr_start_server(link_t* link, const ufr_args_t* args);

/**
 * @brief start a link as socket client
 * 
 * @param link link to be started
 * @param args variable arguments for start function
 * @return int error code or 0 for success
 */
int ufr_start_client(link_t* link, const ufr_args_t* args);

// ============================================================================
//  Stop and Close
// ============================================================================

/**
 * @brief 
 * 
 * @param link 
 */
void ufr_stop(link_t* link);

/**
 * @brief 
 * 
 * @param link 
 */
void ufr_close(link_t* link);


bool ufr_loop_ok();
void ufr_loop_set_end();
int ufr_put_loop_callback( int (*loop_callback)(void)  );

// ============================================================================
//  Receive, Read and Get
// ============================================================================

/**
 * @brief 
 * 
 * @param link 
 * @return int 
 */
int ufr_recv(link_t* link);

/**
 * @brief 
 * 
 * @param link 
 * @return int 
 */
int ufr_recv_async(link_t* link);




int ufr_get_nbytes(link_t* link);
int ufr_get_nitems(link_t* link);




const uint8_t* ufr_get_raw_ptr(link_t* link);

/**
 * @brief 
 * 
 * @param node 
 * @param buffer 
 * @param size 
 * @return size_t 
 */
size_t ufr_read(link_t* node, char* buffer, size_t size);

/**
 * @brief 
 * 
 * @param link 
 * @param format 
 * @param list 
 * @return int 
 */
int ufr_get_va(link_t* link, const char* format, va_list list);

/**
 * @brief 
 * 
 * @param link 
 * @param format 
 * @param ... 
 * @return int 
 */
int ufr_get(link_t* link, const char* format, ...);

/**
 * @brief 
 * 
 * @param link 
 */
void ufr_get_eof(link_t* link);

/**
 * @brief 
 * 
 * @param link 
 * @return char 
 */
char ufr_get_type(link_t* link);

/**
 * @brief 
 * 
 * @param link 
 * @param buffer 
 */
int ufr_get_str(link_t* link, char* buffer, int maxlen);

/**
 * @brief 
 * 
 * @param link 
 * @param buffer 
 * @param maxsize 
 * @return size_t 
 */
int ufr_get_raw(link_t* link, uint8_t buffer[], int max_nitems);


// ============================================================================
//  Send, Write and Put
// ============================================================================

/**
 * @brief 
 * 
 * @param link 
 * @return true 
 * @return false 
 */
bool ufr_send(link_t* link);

/**
 * @brief 
 * 
 * @param node 
 * @param buffer 
 * @param size 
 * @return size_t 
 */
size_t ufr_write(link_t* node, const char* buffer, size_t size);

/**
 * @brief 
 * 
 * @param link 
 * @param format 
 * @param list 
 */
int ufr_put_va(link_t* link, const char* format, va_list list);


/**
 * @brief Put data to the message for link
 *
 * This function put formatted data to the message and it will be send
 * to the link related. 
 *
 * @param link pointer of the link
 * @param format format of the message (i: integer, s:string, f: float, \\n: send package)
 * @param ...  data for the format
 */
int ufr_put(link_t* link, const char* format, ...);

// PUT
int ufr_put_pu32(link_t* link, const uint32_t* array, int nitems);
int ufr_put_pi32(link_t* link, const int32_t* array, int nitems);
int ufr_put_pf32(link_t* link, const float* array, int nitems);

int ufr_put_pu64(link_t* link, const uint64_t* array, int nitems);
int ufr_put_pi64(link_t* link, const int64_t* array, int nitems);
int ufr_put_pf64(link_t* link, const double* array, int nitems);

int ufr_put_raw(link_t* link, const uint8_t* array, int nbytes);
int ufr_put_u8(link_t* link, const uint8_t* array, int nbytes);
int ufr_put_i8(link_t* link, const int8_t* array, int nbytes);


int ufr_put_str(link_t* link, const char* value);
int ufr_put_eof(link_t* link);


int ufr_put_enter(link_t* link, int max_nitems);
int ufr_put_leave(link_t* link);


// GET Scalar - 32 bites 
uint32_t ufr_get_u32(link_t* link, uint32_t defval);
int32_t  ufr_get_i32(link_t* link, int32_t defval);
float    ufr_get_f32(link_t* link, float defval);

// GET Vector - 64 bites
int ufr_get_pf32(link_t* link, float buffer[], int max_nitems);

// GET Scalar - 64 bites
uint64_t ufr_get_u64(link_t* link, uint64_t defval);
int64_t  ufr_get_i64(link_t* link, int64_t defval);
double   ufr_get_f64(link_t* link, double defval);

// Enter and Leave
int ufr_get_enter(link_t* link);
int ufr_get_leave(link_t* link);



// Functions on ufr_args_t
bool ufr_flex_text_div(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max, const char div);
bool ufr_flex_text(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max);

size_t ufr_args_getu(const ufr_args_t* args, const char* noun, const size_t default_value);
int    ufr_args_geti(const ufr_args_t* args, const char* noun, const int default_value);
float  ufr_args_getf(const ufr_args_t* args, const char* noun, const float default_value);
const void* ufr_args_getp(const ufr_args_t* args, const char* noun, const void* default_value);
const char* ufr_args_gets(const ufr_args_t* args, const char* noun, const char* default_value);




const char* ufr_test_args(const link_t* link);

link_t ufr_accept(link_t* link);

int ufr_recv_peer_name(link_t* link, char* buffer, size_t maxbuffer);



link_t ufr_sys_subscriber22(const char* name);
link_t ufr_sys_publisher22(const char* name, const char* params);

int ufr_recv_2s(link_t* link0, link_t* link1, int time_ms);
int ufr_recv_2a(link_t* link0, link_t* link1, int time_ms);

int ufr_recv_3a(link_t* link0, link_t* link1, link_t* link2, int time_ms);

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

// Functions used for test framework
int ufr_load_gtw_from_lib(link_t* link, const char* lib_file, const char* pack_name);



// ufr_sys.h
int ufr_load_gateway(link_t* link, const char* class_name, const ufr_args_t* args);
int ufr_load_encoder(link_t* link, const char* class_name, const ufr_args_t* args);
int ufr_load_decoder(link_t* link, const char* class_name, const ufr_args_t* args);
int ufr_new_ptr(link_t* link, const char* text);

int ufr_link_with_type(link_t* link, const char* text, int boot_type);

link_t ufr_sys_publisher(const char* var_name, const char* text);
link_t ufr_sys_subscriber(const char* name, const char* default_text);

void ufr_output_init(const char* text);
void ufr_output(const char* format, ...);

void ufr_input_init(const char* text);
void ufr_input(const char* format, ...);
bool ufr_input_recv();

void ufr_inoutput_init(const char* text);


// ============================================================================
//  Log Functions
// ============================================================================

void ufr_put_log(link_t* link, uint8_t level, const char* func_name, const char* format, ...);
int  ufr_put_log_error(link_t* link, int error, const char* func_name, const char* format, ...);
int  ufr_put_log_error_ident(link_t* link, int error, const char* func_name, const char* format, ...);

#define ufr_warn(link, ...) ufr_put_log(link, 1, __func__, __VA_ARGS__)
#define ufr_info(link, ...) ufr_put_log(link, 2, __func__, __VA_ARGS__)
#define ufr_log(link, ...) ufr_put_log(link, 2, __func__, __VA_ARGS__)
#define ufr_log_end(link, ...) if (link->log_ident > 0){link->log_ident-=1;} ufr_put_log(link, 3, __func__, __VA_ARGS__)
#define ufr_log_ini(link, ...) ufr_put_log(link, 4, __func__, __VA_ARGS__); link->log_ident+=1
#define ufr_log_error(link, error, ...) ufr_put_log_error_ident(link, error, __func__, __VA_ARGS__);

#define ufr_error(link, error, ...) ufr_put_log_error(link, error, __func__, __VA_ARGS__)
#define ufr_fatal(link, error, ...) ufr_put_log_error(link, error, __func__, __VA_ARGS__); exit(1)


// ============================================================================
//  Functions with dependency of Operating System
// ============================================================================

link_t ufr_sys_open(const char* name, const char* def_args);
const char* ufr_sys_lib_call_list (const uint8_t slot, const uint8_t list_idx);
int ufr_sys_lib_call_new (link_t* link, const uint8_t slot, const char* name, const int type);
void urf_sys_set_ld_path(char* path);

// ============================================================================
//  Buffer Implementation for Encoders
// ============================================================================

#define MESSAGE_ITEM_SIZE 4096

typedef struct {
    size_t size;
    size_t max;
    char* ptr;
} ufr_buffer_t;

ufr_buffer_t* ufr_buffer_new();
void ufr_buffer_init(ufr_buffer_t* buffer);
void ufr_buffer_clear(ufr_buffer_t* buffer);
void ufr_buffer_free(ufr_buffer_t* buffer);
void ufr_buffer_put(ufr_buffer_t* buffer, const char* text, size_t size);
void ufr_buffer_put_chr(ufr_buffer_t* buffer, char val);
void ufr_buffer_put_u8_as_str(ufr_buffer_t* buffer, uint8_t val);
void ufr_buffer_put_i8_as_str(ufr_buffer_t* buffer, int8_t val);
void ufr_buffer_put_u32_as_str(ufr_buffer_t* buffer, uint32_t val);
void ufr_buffer_put_i32_as_str(ufr_buffer_t* buffer, int32_t val);
void ufr_buffer_put_f32_as_str(ufr_buffer_t* buffer, float val);
void ufr_buffer_put_str(ufr_buffer_t* buffer, const char* text);
void ufr_buffer_check_size(ufr_buffer_t* buffer, size_t size);




int sys_ufr_load (link_t* link, const char* library_type, 
    const char* class_path, int boot_type, const ufr_args_t* args);



// ============================================================================
//  Use for tests
// ============================================================================

int ufr_gtw_posix_new_pipe(link_t* link, int type);
int ufr_dcr_sys_new_std(link_t* link, int type);
int ufr_enc_sys_new_std(link_t* link, int type);
link_t ufr_new_pipe();

void ufr_test_inc_count();
void ufr_test_print_result();


// ============================================================================
//  App
// ============================================================================

int sys_ufr_new_link(link_t* link, int boot_type, const ufr_args_t* args);

int ufr_app_init(ufr_node_t* root);
int ufr_app_open(link_t* link, const char* name, int type);
link_t ufr_app_publisher(const char* name);
link_t ufr_app_subscriber(const char* name);
link_t ufr_app_client(const char* name);
link_t ufr_app_server(const char* name);

// ============================================================================
//  Footer
// ============================================================================

#ifdef __cplusplus
}
#endif
