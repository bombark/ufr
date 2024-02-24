// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include "ufr.h"

// ============================================================================
//  Gateway
// ============================================================================

int    ufr_gtw_pack_type(const link_t* link) {

}

int    ufr_gtw_pack_state(const link_t* link) {

}

size_t ufr_gtw_pack_size(const link_t* link, int type) {

}

int  ufr_gtw_pack_boot(link_t* link, const ufr_args_t* args) {

}

int  ufr_gtw_pack_start(link_t* link, int type, const ufr_args_t* args) {

}

void ufr_gtw_pack_stop(link_t* link, int type) {

}

int  ufr_gtw_pack_copy(link_t* link, link_t* out) {

}

size_t ufr_gtw_pack_read(link_t* link, char* buffer, size_t length) {

}

size_t ufr_gtw_pack_write(link_t* link, const char* buffer, size_t length) {

}

bool ufr_gtw_pack_recv(link_t* link) {

}

bool ufr_gtw_pack_recv_async(link_t* link) {

}

int ufr_gtw_pack_send(link_t* link) {

}

int ufr_gtw_pack_accept(link_t* link, link_t* out_client) {

}

// tests
const char* ufr_gtw_pack_test_args(const link_t* link) {

}

ufr_gtw_api_t ufr_gtw_pack_api = {
	.type = ufr_gtw_pack_type,
	.state = ufr_gtw_pack_state,
	.size = ufr_gtw_pack_size,

	.boot = ufr_gtw_pack_boot,
	.start = ufr_gtw_pack_start,
	.stop = ufr_gtw_pack_stop,
	.copy = ufr_gtw_pack_copy,

	.read = ufr_gtw_pack_read,
	.write = ufr_gtw_pack_write,

	.recv = ufr_gtw_pack_recv,
	.recv_async = ufr_gtw_pack_recv_async,

    .send = ufr_gtw_pack_send,
    .accept = ufr_gtw_pack_accept,

    .test_args = ufr_gtw_pack_test_args,
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_gtw_pack_new_std(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_pack_api;
    return UFR_OK;
}