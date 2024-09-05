// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include "ufr.h"

// ============================================================================
//  Decoder
// ============================================================================

static
int ufr_dcr_pack_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_dcr_pack_close(link_t* link) {

}

static
void ufr_dcr_pack_recv_cb(link_t* link, char* msg_data, size_t msg_size) {

}

static
int ufr_dcr_pack_get_u32(link_t* link, uint32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_pack_get_i32(link_t* link, int32_t* val) {
    return UFR_OK;
}

static
int ufr_dcr_pack_get_f32(link_t* link, float* ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_pack_get_str(link_t* link, char** ret_val) {
    return UFR_OK;
}

static
int ufr_dcr_pack_get_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_pack_copy_str(link_t* link, char* ret_val, size_t size_max) {
    return UFR_OK;
}

static
int ufr_dcr_pack_copy_arr(link_t* link, char arr_type, size_t arr_size_max, size_t* arr_size, void* arr_ptr) {
    return UFR_OK;
}

static
int ufr_dcr_pack_enter_array(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_pack_leave_array(link_t* link) {
    return UFR_OK;
}

static
ufr_dcr_api_t dcr_pack_api = {
    .boot = ufr_dcr_pack_boot,
    .close = ufr_dcr_pack_close,
	.recv_cb = ufr_dcr_pack_recv_cb,

	.get_u32 = ufr_dcr_pack_get_u32,
	.get_i32 = ufr_dcr_pack_get_i32,
	.get_f32 = ufr_dcr_pack_get_f32,
	.get_str = ufr_dcr_pack_get_str,
	.get_arr = ufr_dcr_pack_get_arr,

    .copy_str = ufr_dcr_pack_copy_str,
	.copy_arr = ufr_dcr_pack_copy_arr,

    .enter_array = ufr_dcr_pack_enter_array,
    .leave_array = ufr_dcr_pack_leave_array
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_dcr_pack_new_std(link_t* link, int type) {
    link->dcr_api = &dcr_pack_api;
    return UFR_OK;
}