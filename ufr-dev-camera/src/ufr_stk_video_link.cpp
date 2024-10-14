// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


struct GatewayLink {
    link_t link;
    std::vector<uint8_t> buffer;
    Mat frame;
};

// Mover essa funcao para UFR
// @new aaa @param1 bbb @@new ccc @@param1 ddd -> @new ccc @param1 ddd
int ufr_args_decrease_level(const char* src, char* dst) {
    dst[0] = '\0';
    char token[512];
    uint8_t count_arg = 0;
    uint16_t cursor = 0;
    bool ignore = true;
    while( ufr_flex_text(src, &cursor, token, sizeof(token)) ) {
        // uint32_t len = strlen(token);
        // if ( len > 2 ) {
        if ( token[0] == '@' ) {
            if ( token[1] == '@' ) {
                strcat(dst, &token[1]);
                strcat(dst, " ");
                ignore = false;
            } else {
                ignore = true;
            }
        } else {
            if ( ignore == false ) {
                strcat(dst, token);
                strcat(dst, " ");
            }
        }

    }

    return UFR_OK;
}


// ============================================================================
//  Encoder Link
// ============================================================================

static
int ufr_enc_link_boot(link_t* link, const ufr_args_t* args) {
	return UFR_OK;
}

static
void ufr_enc_link_close(link_t* link) {
	if ( link->dcr_obj != NULL ) {
		
	}
}

static
int ufr_enc_link_put_u32(link_t* link, uint32_t val) {
	
	return UFR_OK;
}

static
int ufr_enc_link_put_i32(link_t* link, int32_t val) {
	
	return UFR_OK;
}

static
int ufr_enc_link_put_f32(link_t* link, float val) {
	
	return UFR_OK;
}

static
int ufr_enc_link_put_str(link_t* link, const char* val) {
	
	return UFR_OK;
}

static
int ufr_enc_link_put_arr(link_t* link, const void* arr_ptr, char type, size_t arr_size) {
    return UFR_OK;
}

static
int ufr_enc_link_put_cmd(link_t* link, char cmd) {
	return UFR_OK;
}

static
int ufr_enc_link_put_raw(link_t* link, const uint8_t* buffer, size_t size) {
	return UFR_OK;
}

int ufr_enc_link_enter_array(link_t* link, size_t maxsize) {
    return UFR_OK;
}


int ufr_enc_link_leave_array(link_t* link) {
    return UFR_OK;
}

ufr_enc_api_t ufr_enc_link_api = {
    .boot = ufr_enc_link_boot,
    .close = ufr_enc_link_close,
    .clear = NULL,

    .set_header = NULL,

    .put_u8 = NULL,
    .put_i8 = NULL,
    .put_cmd = ufr_enc_link_put_cmd,
    .put_str = NULL,
    .put_raw = ufr_enc_link_put_raw,

    .put_u32 = ufr_enc_link_put_u32,
    .put_i32 = ufr_enc_link_put_i32,
    .put_f32 = ufr_enc_link_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_arr = ufr_enc_link_put_arr,
    .put_mat = NULL,

    .enter_array = ufr_enc_link_enter_array,
    .leave_array = ufr_enc_link_leave_array
};


// ============================================================================
//  Decoder Link
// ============================================================================

static
int ufr_dcr_link_boot(link_t* link, const ufr_args_t* args) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    return ufr_boot_dcr(&gtw->link, args);
}

static
void ufr_dcr_link_close(link_t* link) {
    
}

static
void ufr_dcr_link_recv_cb(link_t* link, char* msg_data, size_t msg_size) {

}

size_t ufr_dcr_link_get_size(link_t* link) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    return gtw->frame.total();
}

uint8_t* ufr_dcr_link_get_raw_ptr(link_t* link) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    return (uint8_t*) gtw->frame.data;
}

static
int ufr_dcr_link_get_u32(link_t* link, uint32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_link_get_i32(link_t* link, int32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_link_get_f32(link_t* link, float* ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_link_get_str(link_t* link, char** ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_link_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_link_copy_str(link_t* link, char* ret_val, size_t size_max) {
    return UFR_OK;
}

static
int ufr_dcr_link_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_link_enter_array(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_link_leave_array(link_t* link) {
    return UFR_OK;
}

ufr_dcr_api_t ufr_dcr_link_api = {
    .boot = ufr_dcr_link_boot,
    .close = ufr_dcr_link_close,
    .recv_cb = ufr_dcr_link_recv_cb,
    .recv_async_cb = NULL,
    .next = NULL,

    .get_type = NULL,
    .get_size = ufr_dcr_link_get_size,
    .get_raw_ptr = ufr_dcr_link_get_raw_ptr,

    .get_u32 = ufr_dcr_link_get_u32,
    .get_i32 = ufr_dcr_link_get_i32,
    .get_f32 = ufr_dcr_link_get_f32,
    .get_str = ufr_dcr_link_get_str,
    .get_arr = ufr_dcr_link_get_arr,
    .get_ai32 = NULL,

    .copy_str = ufr_dcr_link_copy_str,
    .copy_arr = ufr_dcr_link_copy_arr,

    .enter_array = ufr_dcr_link_enter_array,
    .leave_array = ufr_dcr_link_leave_array
};


// ============================================================================
//  Gateway Link
// ============================================================================

int    ufr_gtw_link_type(const link_t* link) {
    return UFR_OK;
}

int    ufr_gtw_link_state(const link_t* link) {
    return UFR_OK;
}

size_t ufr_gtw_link_size(const link_t* link, int type) {
    return 0;
}

int  ufr_gtw_link_boot(link_t* link, const ufr_args_t* args) {
    GatewayLink* gtw = new GatewayLink();

    char link_text[1024];
    ufr_args_decrease_level(args->text, link_text);
    // printf("%s\n", link_text);

    gtw->link = ufr_subscriber(link_text);
    // gtw->data = malloc(1024*1024);
    link->gtw_obj = (void*) gtw;
    link->gtw_shr = NULL;
    return UFR_OK;
}

int  ufr_gtw_link_start(link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_SUBSCRIBER ) {
        link->dcr_api = &ufr_dcr_link_api;
        ufr_boot_dcr(link, args);
        GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
        return ufr_start(&gtw->link, type, args);
    }

    if ( type == UFR_START_PUBLISHER ) {
        return UFR_OK;
    }

    return 1;
}

void ufr_gtw_link_stop(link_t* link, int type) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    // ufr_stop(&gtw->link);
}

int  ufr_gtw_link_copy(link_t* link, link_t* out) {
    return UFR_OK;
}

size_t ufr_gtw_link_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

size_t ufr_gtw_link_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

int ufr_gtw_link_recv(link_t* link) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    ufr_recv(&gtw->link);

    const size_t size = ufr_get_size(&gtw->link);
    gtw->buffer.resize(size);
    ufr_get_raw(&gtw->link, gtw->buffer.data(), size);
    gtw->frame = imdecode(gtw->buffer, cv::IMREAD_COLOR);

    return UFR_OK;
}

int ufr_gtw_link_recv_async(link_t* link) {
    return UFR_OK;
}

int ufr_gtw_lin_send(link_t* link) {
    return UFR_OK;
}

int ufr_gtw_link_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

// tests
const char* ufr_gtw_link_test_args(const link_t* link) {
    return "";
}

// ============================================================================
//  Public
// ============================================================================

extern "C" {

ufr_gtw_api_t ufr_gtw_link_api = {
    .name = "Camera:Link",

    .type = ufr_gtw_link_type,
    .state = ufr_gtw_link_state,
    .size = ufr_gtw_link_size,

    .boot = ufr_gtw_link_boot,
    .start = ufr_gtw_link_start,
    .stop = ufr_gtw_link_stop,
    .copy = ufr_gtw_link_copy,

    .read = ufr_gtw_link_read,
    .write = ufr_gtw_link_write,

    .recv = ufr_gtw_link_recv,
    .recv_async = ufr_gtw_link_recv_async,
    .recv_peer_name = NULL,

    .accept = ufr_gtw_link_accept,

    .test_args = ufr_gtw_link_test_args,
};

}


