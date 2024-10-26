/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Felipe Bombardelli
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

#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <ufr.h>
#include <unistd.h>

// ============================================================================
//  File
// ============================================================================

static
int ufr_gtw_posix_timer_type(const link_t* link) {
	return 0;
}

static
int ufr_gtw_posix_timer_state(const link_t* link){
	return 0;
}

static
size_t ufr_gtw_posix_timer_size(const link_t* link, int type){
	return 0;
}

static
int ufr_gtw_posix_timer_boot(link_t* link, const ufr_args_t* args) {
	
    // success
	return UFR_OK;
}

static
int ufr_gtw_posix_timer_start(link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_SUBSCRIBER ) {
    
    // error
    } else {
        return ufr_error(link, 1, "parameter type(%d) is invalid", type);
    }

    // success
	return UFR_OK;
}

static
void ufr_gtw_posix_timer_stop(link_t* link, int type) {
    
}

static 
int ufr_gtw_posix_timer_recv(link_t* link) {
    time_t first = time(0);
    time_t now;
    printf("first %ld\n", first);
    do {
        usleep(500000);
        now = time(0);
    } while ( now == first );
    printf("time %ld\n", now);

    return UFR_OK;
}

static
int ufr_gtw_posix_timer_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t ufr_gtw_posix_timer_read(link_t* link, char* buffer, size_t length) {
	return 0;
}

static
size_t ufr_gtw_posix_timer_write(link_t* link, const char* buffer, size_t length) {
	return 0;
}

const char* ufr_gtw_posix_timer_test_args(const link_t* link) {
    return "";
}

static
ufr_gtw_api_t ufr_gtw_posix_timer_api = {
    .name = "file",
    .type = ufr_gtw_posix_timer_type,
    .state = ufr_gtw_posix_timer_state,
    .size = ufr_gtw_posix_timer_size,
    .boot = ufr_gtw_posix_timer_boot,
    .start = ufr_gtw_posix_timer_start,
    .stop = ufr_gtw_posix_timer_stop,
    .copy = ufr_gtw_posix_timer_copy,
    .read = ufr_gtw_posix_timer_read,
    .write = ufr_gtw_posix_timer_write,
    .recv = ufr_gtw_posix_timer_recv,
    .test_args = ufr_gtw_posix_timer_test_args
};


// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_posix_new_timer(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_posix_timer_api;
    link->type_started = type;
    return UFR_OK;
}