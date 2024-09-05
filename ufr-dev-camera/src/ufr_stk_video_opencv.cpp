// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

struct Gateway {
    VideoCapture capture;
    Mat frame;
};

// ============================================================================
//  Decoder OpenCV
// ============================================================================

static
int ufr_dcr_opencv_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_dcr_opencv_close(link_t* link) {

}

static
void ufr_dcr_opencv_recv_cb(link_t* link, char* msg_data, size_t msg_size) {

}

static
size_t ufr_dcr_opencv_get_size(link_t* link) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    return gtw->frame.total();
}

static
uint8_t* ufr_dcr_opencv_get_raw_ptr(link_t* link) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    return (uint8_t*) gtw->frame.data;
}

static
int ufr_dcr_opencv_get_u32(link_t* link, uint32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_get_i32(link_t* link, int32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_get_f32(link_t* link, float* ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_get_str(link_t* link, char** ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_copy_str(link_t* link, char* ret_val, size_t size_max) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_enter_array(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_leave_array(link_t* link) {
    return UFR_OK;
}

static
ufr_dcr_api_t ufr_dcr_opencv_api = {
    .boot = ufr_dcr_opencv_boot,
    .close = ufr_dcr_opencv_close,
	.recv_cb = ufr_dcr_opencv_recv_cb,

    .get_size = ufr_dcr_opencv_get_size,
    .get_raw_ptr = ufr_dcr_opencv_get_raw_ptr,

	.get_u32 = ufr_dcr_opencv_get_u32,
	.get_i32 = ufr_dcr_opencv_get_i32,
	.get_f32 = ufr_dcr_opencv_get_f32,
	.get_str = ufr_dcr_opencv_get_str,
	.get_arr = ufr_dcr_opencv_get_arr,

    .copy_str = ufr_dcr_opencv_copy_str,
	.copy_arr = ufr_dcr_opencv_copy_arr,

    .enter_array = ufr_dcr_opencv_enter_array,
    .leave_array = ufr_dcr_opencv_leave_array
};


// ============================================================================
//  Gateway OpenCV
// ============================================================================

static
int    ufr_gtw_opencv_type(const link_t* link) {
    return UFR_OK;
}

static
int    ufr_gtw_opencv_state(const link_t* link) {
    return UFR_OK;
}

static
size_t ufr_gtw_opencv_size(const link_t* link, int type) {
    return 0;
}

static
int  ufr_gtw_opencv_boot(link_t* link, const ufr_args_t* args) {
    Gateway* gtw = new Gateway();
    link->gtw_obj = gtw;
    link->gtw_shr = NULL;
    return UFR_OK;
}

static
int  ufr_gtw_opencv_start(link_t* link, int type, const ufr_args_t* args) {
    Gateway* gtw = (Gateway*) link->gtw_obj;

    link->dcr_api = &ufr_dcr_opencv_api;
    ufr_boot_dcr(link, args);

    int idx = ufr_args_geti(args, "@id", 0);
    gtw->capture.open(idx);
    return UFR_OK;
}

static
void ufr_gtw_opencv_stop(link_t* link, int type) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    gtw->capture.release();
}

static
int  ufr_gtw_opencv_copy(link_t* link, link_t* out) {
    return UFR_OK;
}

static
size_t ufr_gtw_opencv_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t ufr_gtw_opencv_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int ufr_gtw_opencv_recv(link_t* link) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    gtw->capture >> gtw->frame;
    return UFR_OK;
}

static
int ufr_gtw_opencv_recv_async(link_t* link) {
    return UFR_OK;
}

static
int ufr_gtw_opencv_send(link_t* link) {
    return UFR_OK;
}

static
int ufr_gtw_opencv_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

static
const char* ufr_gtw_opencv_test_args(const link_t* link) {
    return "";
}

extern "C"
ufr_gtw_api_t ufr_gtw_opencv_api = {
	.type = ufr_gtw_opencv_type,
	.state = ufr_gtw_opencv_state,
	.size = ufr_gtw_opencv_size,

	.boot = ufr_gtw_opencv_boot,
	.start = ufr_gtw_opencv_start,
	.stop = ufr_gtw_opencv_stop,
	.copy = ufr_gtw_opencv_copy,

	.read = ufr_gtw_opencv_read,
	.write = ufr_gtw_opencv_write,

	.recv = ufr_gtw_opencv_recv,
	.recv_async = ufr_gtw_opencv_recv_async,

    .accept = ufr_gtw_opencv_accept,

    .test_args = ufr_gtw_opencv_test_args,
};

