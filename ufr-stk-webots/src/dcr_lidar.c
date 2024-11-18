// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <ufr.h>

#include "ufr_webots.h"

typedef struct {
    WbDeviceTag lidar;
    float const* values_ptr;
    uint32_t values_size;
    int max_range;
    uint16_t index;
} decoder_t;

// ============================================================================
//  Decoder
// ============================================================================

static
int ufr_dcr_lidar_boot(link_t* link, const ufr_args_t* args) {
    // new decoder
    decoder_t* dcr = malloc(sizeof(decoder_t));
    dcr->index = 0;
    dcr->values_ptr = NULL;

    // get sensor for both wheel
    const char* dev_tag_name = ufr_args_gets(args, "@tag", "RPlidar A2");
    const int time_step = ufr_gtw_webots_get_time_step();
    dcr->lidar = wb_robot_get_device(dev_tag_name);
    dcr->values_size = 0;
    dcr->max_range = 0;
    wb_lidar_enable(dcr->lidar, time_step);

    // success
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_lidar_close(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr ) {
        free(dcr);
        link->dcr_obj = NULL;
    }
}

static
char ufr_dcr_lidar_get_type(link_t* link) {
    return 'a';
}

static
int ufr_dcr_lidar_get_nitems(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr ) {
        return dcr->values_size;
    }
    return 0;
}

static
int ufr_dcr_lidar_get_nbytes(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr ) {
        return dcr->values_size * sizeof(float);
    }
    return 0;
}

static
uint8_t* ufr_dcr_lidar_get_raw_ptr(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    return (uint8_t*) dcr->values_ptr;
}

static
int ufr_dcr_lidar_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr ) {
        dcr->values_ptr = wb_lidar_get_range_image(dcr->lidar);
        dcr->values_size = wb_lidar_get_horizontal_resolution(dcr->lidar);
        dcr->max_range = wb_lidar_get_max_range(dcr->lidar);
        dcr->index = 0;
    }
    return UFR_OK;
}

static
int ufr_dcr_lidar_get_u32(link_t* link, uint32_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->values_size ) {
            val[wrote] = dcr->values_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_i32(link_t* link, int32_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->values_size ) {
            val[wrote] = dcr->values_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_f32(link_t* link, float val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->values_size ) {
            val[wrote] = dcr->values_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_u64(link_t* link, uint64_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->values_size ) {
            val[wrote] = dcr->values_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_i64(link_t* link, int64_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->values_size ) {
            val[wrote] = dcr->values_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_f64(link_t* link, double val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->values_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->values_size ) {
            val[wrote] = dcr->values_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}


static
int ufr_dcr_lidar_get_str(link_t* link, char* ret_val, int size) {
    return UFR_OK;
}

static
int ufr_dcr_lidar_enter(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_lidar_leave(link_t* link) {
    return UFR_OK;
}

static
ufr_dcr_api_t dcr_lidar_api = {
    .boot = ufr_dcr_lidar_boot,
    .close = ufr_dcr_lidar_close,
	.recv_cb = ufr_dcr_lidar_recv_cb,
    .recv_async_cb = ufr_dcr_lidar_recv_cb,

    .get_type = ufr_dcr_lidar_get_type,
    .get_nbytes = ufr_dcr_lidar_get_nbytes,
    .get_nitems = ufr_dcr_lidar_get_nitems,
    .get_raw_ptr = ufr_dcr_lidar_get_raw_ptr,

	.get_u32 = ufr_dcr_lidar_get_u32,
	.get_i32 = ufr_dcr_lidar_get_i32,
	.get_f32 = ufr_dcr_lidar_get_f32,

	.get_u64 = ufr_dcr_lidar_get_u64,
	.get_i64 = ufr_dcr_lidar_get_i64,
	.get_f64 = ufr_dcr_lidar_get_f64,

	.get_str = ufr_dcr_lidar_get_str,

    .enter = ufr_dcr_lidar_enter,
    .leave = ufr_dcr_lidar_leave
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_dcr_webots_new_lidar(link_t* link, int type) {
    link->dcr_api = &dcr_lidar_api;
    return UFR_OK;
}