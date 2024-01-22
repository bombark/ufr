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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <ufr.h>

typedef struct {
    pthread_t id;
    int (*main)(int argc, char** argv);
} ll_thread_t;

// ============================================================================
//  PThread
// ============================================================================

static
int lt_posix_thread_type(const link_t* link) {
	return 0;
}

static
int lt_posix_thread_state(const link_t* link){
	return 0;
}

static
size_t lt_posix_thread_size(const link_t* link, int type){
	return 0;
}

static
int lt_posix_thread_boot(link_t* link, const lt_args_t* args) {
    int (*main)(int,char**) = (int(*)(int,char**)) lt_args_getp(args, "@main");
    assert( main!=NULL );

    ll_thread_t* thread = malloc(sizeof(ll_thread_t));
    thread->id = 0;
    thread->main = main;
    link->gw_obj = thread;
	return 0;
}

static
int lt_posix_thread_start(link_t* link, int type, const lt_args_t* args) {
    ll_thread_t* thread = (ll_thread_t*) link->gw_obj;

    int error = pthread_create(thread->id, NULL, thread->main, NULL);
	return 0;
}

static
void lt_posix_thread_stop(link_t* link, int type) {
    
}

static
int lt_posix_thread_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t lt_posix_thread_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t lt_posix_thread_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
lt_api_t lt_posix_thread = {
	.type = lt_posix_thread_type,
	.state = lt_posix_thread_state,
	.size = lt_posix_thread_size,
	.boot = lt_posix_thread_boot,
	.start = lt_posix_thread_start,
	.stop = lt_posix_thread_stop,
	.copy = lt_posix_thread_copy,
	.read = lt_posix_thread_read,
	.write = lt_posix_thread_write,
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_new_gtw_posix_thread(link_t* link, const lt_args_t* args) {
	link->gw_api = &lt_posix_thread;
	lt_posix_thread_boot(link, args);
	return LT_OK;
}
