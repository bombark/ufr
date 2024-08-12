/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica e Imagem (VRI)
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
//  HEADER
// ============================================================================

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "ufr.h"

int ufr_dcr_sys_new_std(link_t* link, int type);

// ============================================================================
//  Fake Gateway
// ============================================================================

int    ufr_gtw_fake_type(const link_t* link) {

}

int    ufr_gtw_fake_state(const link_t* link) {

}

size_t ufr_gtw_fake_size(const link_t* link, int type) {

}

int  ufr_gtw_fake_boot(link_t* link, const ufr_args_t* args) {
	return UFR_OK;
}

int  ufr_gtw_fake_start(link_t* link, int type, const ufr_args_t* args) {
	return UFR_OK;
}

void ufr_gtw_fake_stop(link_t* link, int type) {

}

int  ufr_gtw_fake_copy(link_t* link, link_t* out) {

}

size_t ufr_gtw_fake_read(link_t* link, char* buffer, size_t length) {

}

size_t ufr_gtw_fake_write(link_t* link, const char* buffer, size_t length) {

}

int ufr_gtw_fake_recv(link_t* link) {

}

int ufr_gtw_fake_recv_async(link_t* link) {

}

int ufr_gtw_fake_accept(link_t* link, link_t* out_client) {

}

// tests
const char* ufr_gtw_fake_test_args(const link_t* link) {

}

ufr_gtw_api_t ufr_gtw_fake_api = {
	.name = "fake",

	.type = ufr_gtw_fake_type,
	.state = ufr_gtw_fake_state,
	.size = ufr_gtw_fake_size,

	.boot = ufr_gtw_fake_boot,
	.start = ufr_gtw_fake_start,
	.stop = ufr_gtw_fake_stop,
	.copy = ufr_gtw_fake_copy,

	.read = ufr_gtw_fake_read,
	.write = ufr_gtw_fake_write,

	.recv = ufr_gtw_fake_recv,
	.recv_async = ufr_gtw_fake_recv_async,

    .accept = ufr_gtw_fake_accept,

    .test_args = ufr_gtw_fake_test_args,
};

int ufr_gtw_fake_new_std(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_fake_api;
    return UFR_OK;
}

// ============================================================================
//  Main
// ============================================================================

void test_init_link() {
	link_t link;
	ufr_init_link(&link, NULL);
	assert( link.gtw_api == NULL );
	assert( link.dcr_api == NULL );
	assert( link.dcr_obj == NULL );
	assert( link.enc_api == NULL );
	assert( link.enc_obj == NULL );
}

void test_get_api_name() {
	{
		const char* name = ufr_api_name(NULL);
		assert( strcmp(name, "None") == 0 );
	}

	{
		link_t link;
		ufr_init_link(&link, NULL);
		const char* name = ufr_api_name(&link);
		assert( strcmp(name, "None") == 0 );
	}

	{
		link_t link;
		ufr_init_link(&link, &ufr_gtw_fake_api);
		const char* name = ufr_api_name(&link);
		assert( strcmp(name, "fake") == 0 );
	}
}

void test_publisher() {
	link_t link;
	assert( ufr_gtw_fake_new_std(&link, UFR_START_PUBLISHER) == UFR_OK);
	assert( link.gtw_api == &ufr_gtw_fake_api );

	const ufr_args_t args = {.text=""};
	assert( ufr_start_publisher(&link, &args) == UFR_OK );


}


void test_writer() {
	link_t decoder;
	ufr_dcr_sys_new_std(&decoder, 0);
}

int main() {
	test_init_link();
	test_get_api_name();

	test_publisher();

	// test_writer();
	return 0;
}