// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include "ufr.h"

// ============================================================================
//  Encoder
// ============================================================================

int ufr_enc_pack_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

void ufr_enc_pack_close(link_t* link) {
}

void ufr_enc_pack_clear(link_t* link) {
}

int ufr_enc_pack_set_header(link_t* link, const char* header) {
    return UFR_OK;
}


int ufr_enc_pack_put_u32(link_t* link, uint32_t val) {
    return UFR_OK;
}

int ufr_enc_pack_put_i32(link_t* link, int32_t val) {
    return UFR_OK;
}

int ufr_enc_pack_put_f32(link_t* link, float val) {
    return UFR_OK;
}

int ufr_enc_pack_put_str(link_t* link, const char* val) {
    return UFR_OK;
}

int ufr_enc_pack_put_cmd(link_t* link, char cmd) {
    return UFR_OK;
}


int ufr_enc_pack_put_arr(link_t* link, const void* array, char type, size_t size) {
    return UFR_OK;
}

int ufr_enc_pack_put_mat(link_t* link, const void* vet, char type, size_t rows, size_t cols) {
    return UFR_OK;
}


int ufr_enc_pack_enter_array(link_t* link, size_t maxsize) {
    return UFR_OK;
}

int ufr_enc_pack_leave_array(link_t* link) {
    return UFR_OK;
}

ufr_enc_api_t ufr_enc_pack_api = {
    .boot = ufr_enc_pack_boot,
    .close = ufr_enc_pack_close,
    .clear = ufr_enc_pack_clear,
    .set_header = ufr_enc_pack_set_header,

    .put_u32 = ufr_enc_pack_put_u32,
    .put_i32 = ufr_enc_pack_put_i32,
    .put_f32 = ufr_enc_pack_put_f32,
    .put_str = ufr_enc_pack_put_str,
    .put_cmd = ufr_enc_pack_put_cmd,

    .put_arr = ufr_enc_pack_put_arr,
    .put_mat = ufr_enc_pack_put_mat,

    .enter_array = ufr_enc_pack_enter_array,
    .leave_array = ufr_enc_pack_leave_array,
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_enc_pack_new_std(link_t* link, int type) {
    link->enc_api = &ufr_enc_pack_api;
    return UFR_OK;
}