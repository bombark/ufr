// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <ufr.h>

#include "ufr_webots.h"

typedef struct {
    WbDeviceTag left;
    WbDeviceTag right;
    float wheel_left, wheel_right;
    uint8_t index;
} dcr_encoders_t;

// ============================================================================
//  Decoder
// ============================================================================

static
int ufr_dcr_encoder_boot(link_t* link, const ufr_args_t* args) {
    // new decoder
    dcr_encoders_t* dcr = malloc(sizeof(dcr_encoders_t));
    dcr->index = 0;

    // get sensor for both wheels
    dcr->left = wb_robot_get_device("left wheel sensor");
    dcr->right = wb_robot_get_device("right wheel sensor");

    // enable the encoders
    const int time_step = ufr_gtw_webots_get_time_step();
    wb_position_sensor_enable(dcr->left, time_step);
    wb_position_sensor_enable(dcr->right, time_step);

    // success
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_encoder_close(link_t* link) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    if ( dcr ) {
        free(dcr);
        link->dcr_obj = NULL;
    }
}

static
int ufr_dcr_encoder_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    if ( dcr ) {
        dcr->wheel_left = wb_position_sensor_get_value(dcr->left);
        dcr->wheel_right = wb_position_sensor_get_value(dcr->right);
        dcr->index = 0;
    } else {
        ufr_fatal(link, 1, "dcr is not booted");
    }
    return UFR_OK;
}

static
int ufr_dcr_encoder_get_u32(link_t* link, uint32_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->wheel_left; break;
            case 1: val[wrote] = dcr->wheel_right; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_encoder_get_i32(link_t* link, int32_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->wheel_left; break;
            case 1: val[wrote] = dcr->wheel_right; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_encoder_get_f32(link_t* link, float* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    if ( dcr ) {
        for (; wrote<nitems; wrote++) {
            switch (dcr->index) {
                case 0: val[wrote] = dcr->wheel_left; break;
                case 1: val[wrote] = dcr->wheel_right; break;
                default: val[wrote] = 0; break;
            }
            dcr->index += 1;
        }
    } else {
        return ufr_error(link, 1, "Decoder is null");
    }
    return wrote;
}

static
int ufr_dcr_encoder_get_u64(link_t* link, uint64_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->wheel_left; break;
            case 1: val[wrote] = dcr->wheel_right; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_encoder_get_i64(link_t* link, int64_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->wheel_left; break;
            case 1: val[wrote] = dcr->wheel_right; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_encoder_get_f64(link_t* link, double* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    if ( dcr ) {
        for (; wrote<nitems; wrote++) {
            switch (dcr->index) {
                case 0: val[wrote] = dcr->wheel_left; break;
                case 1: val[wrote] = dcr->wheel_right; break;
                default: val[wrote] = 0; break;
            }
            dcr->index += 1;
        }
    } else {
        return ufr_error(link, 1, "Decoder is null");
    }
    return wrote;
}

static
int ufr_dcr_encoder_get_str(link_t* link, char* ret_val, int size) {
    return UFR_OK;
}

static
int ufr_dcr_encoder_enter(link_t* link) {
    return -1;
}

static
int ufr_dcr_encoder_leave(link_t* link) {
    return -1;
}

static
ufr_dcr_api_t dcr_encoder_api = {
    .boot = ufr_dcr_encoder_boot,
    .close = ufr_dcr_encoder_close,
	.recv_cb = ufr_dcr_encoder_recv_cb,
    .recv_async_cb = ufr_dcr_encoder_recv_cb,

	.get_u32 = ufr_dcr_encoder_get_u32,
	.get_i32 = ufr_dcr_encoder_get_i32,
	.get_f32 = ufr_dcr_encoder_get_f32,

	.get_u64 = ufr_dcr_encoder_get_u64,
	.get_i64 = ufr_dcr_encoder_get_i64,
	.get_f64 = ufr_dcr_encoder_get_f64,

	.get_str = ufr_dcr_encoder_get_str,

    .enter = ufr_dcr_encoder_enter,
    .leave = ufr_dcr_encoder_leave
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_dcr_webots_new_encoders(link_t* link, int type) {
    link->dcr_api = &dcr_encoder_api;
    return UFR_OK;
}