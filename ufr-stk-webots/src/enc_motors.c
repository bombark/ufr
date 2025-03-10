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

#include <ufr.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <string.h>


#include <stdio.h>

typedef struct {
    WbDeviceTag left;
    WbDeviceTag right;
    double vel, rotvel;
    uint8_t index;
} enc_motors_t;

// ============================================================================
//  Encoder
// ============================================================================

static
int ufr_enc_motors_boot(link_t* link, const ufr_args_t* args) {
    const char* dev_tag1_name = ufr_args_gets(args, "@tag1", "left wheel");
    const char* dev_tag2_name = ufr_args_gets(args, "@tag2", "right wheel");

    // Prepare encoder
    enc_motors_t* enc = malloc(sizeof(enc_motors_t));
    enc->left = wb_robot_get_device( dev_tag1_name );
    enc->right = wb_robot_get_device( dev_tag2_name );
    enc->vel = 0.0;
    enc->rotvel = 0.0;
    enc->index = 0;    

    // Start the WeBots encoders
    wb_motor_set_position(enc->left, INFINITY);
    wb_motor_set_position(enc->right, INFINITY);
    wb_motor_set_velocity(enc->left, 0.0);
    wb_motor_set_velocity(enc->right, 0.0);

    // Success
    link->enc_obj = enc;
    return UFR_OK;
}

static
void ufr_enc_motors_close(link_t* link) {
    if ( link->enc_obj ) {
        free(link->enc_obj);
    }
}

static
void ufr_enc_motors_clear(link_t* link) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    enc->vel = 0.0;
    enc->rotvel = 0.0;
    enc->index = 0;
}

static
int ufr_enc_motors_put_u32(link_t* link, const uint32_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_motors_put_i32(link_t* link, const int32_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_motors_put_f32(link_t* link, const float val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_u64(link_t* link, const uint64_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_motors_put_i64(link_t* link, const int64_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_motors_put_f64(link_t* link, const double val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = val[wrote]; break;
            case 1: enc->rotvel = val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_str(link_t* link, const char* val_str) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    const double val = atof(val_str);
    switch (enc->index) {
        case 0: enc->vel = val; break;
        case 1: enc->rotvel = val; break;
        default: break;
    }
    enc->index += 1;
    return UFR_OK;
}

static
int ufr_enc_motors_put_cmd(link_t* link, char cmd) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    if ( cmd == '\n' ) {
        const double speed_left = enc->vel - enc->rotvel * 0.125; // HALF_DISTANCE_BETWEEN_WHEELS
        const double speed_right = enc->vel + enc->rotvel * 0.125; // HALF_DISTANCE_BETWEEN_WHEELS
// printf("%f %f\n", speed_left, speed_right);
        wb_motor_set_velocity(enc->left, speed_left);
        wb_motor_set_velocity(enc->right, speed_right);
        ufr_enc_motors_clear(link);
    }
    return UFR_OK;
}

static
int ufr_enc_motors_enter(link_t* link, size_t maxsize) {
    return -1;
}

static
int ufr_enc_motors_leave(link_t* link) {
    return -1;
}

static
ufr_enc_api_t ufr_enc_motors_api = {
    .boot = ufr_enc_motors_boot,
    .close = ufr_enc_motors_close,
    .clear = ufr_enc_motors_clear,

    .put_u32 = ufr_enc_motors_put_u32,
    .put_i32 = ufr_enc_motors_put_i32,
    .put_f32 = ufr_enc_motors_put_f32,

    .put_u64 = ufr_enc_motors_put_u64,
    .put_i64 = ufr_enc_motors_put_i64,
    .put_f64 = ufr_enc_motors_put_f64,

    .put_str = ufr_enc_motors_put_str,
    .put_cmd = ufr_enc_motors_put_cmd,

    .enter = ufr_enc_motors_enter,
    .leave = ufr_enc_motors_leave,
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_enc_webots_new_motors(link_t* link, int type) {
    link->enc_api = &ufr_enc_motors_api;
    return UFR_OK;
}