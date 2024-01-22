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
#include <curl/curl.h>

#include "lt_api.h"

// ============================================================================
//  File
// ============================================================================

static
int lt_http_socket_type(const link_t* link) {
	return 0;
}

static
int lt_http_socket_state(const link_t* link){
	return 0;
}

static
size_t lt_http_socket_size(const link_t* link, int type){
	return 0;
}

static
int lt_http_socket_boot(link_t* link, const lt_args_t* args) {
	const char* url = lt_args_gets(args, "@url");
    // struct curl_slist *host = curl_slist_append(NULL, "google.com.br");

    CURL* curl = curl_easy_init();
    link->obj = curl;
    
	return LT_OK;
}

static
int lt_http_socket_start(link_t* link, int type, const lt_args_t* args) {

    if ( type == LT_START_CONNECT ) {
        CURL* curl = (CURL*) link->obj;    
        if(curl) {
            curl_easy_setopt(curl, CURLOPT_URL, "https://example.com");
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        }
    }

	return 0;
}

static
void lt_http_socket_stop(link_t* link, int type) {
    CURL* curl = (CURL*) link->obj;
    curl_easy_cleanup(curl);
}

static
int lt_http_socket_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t lt_http_socket_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t lt_http_socket_write(link_t* link, const char* buffer, size_t length) {
    CURL* curl = (CURL*) link->obj;
    CURLcode res = curl_easy_perform(curl);
    return 0;
}

static
size_t lt_http_socket_recv(link_t* link) {
    CURL* curl = (CURL*) link->obj;
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
    return 0;
}

static
lt_api_t lt_http_socket = {
	.type = lt_http_socket_type,
	.state = lt_http_socket_state,
	.size = lt_http_socket_size,
	.boot = lt_http_socket_boot,
	.start = lt_http_socket_start,
	.stop = lt_http_socket_stop,
	.copy = lt_http_socket_copy,
	.read = lt_http_socket_read,
	.write = lt_http_socket_write,
    .recv = lt_http_socket_recv,
};

int lt_new_http_socket(link_t* link, const lt_args_t* args) {
	link->api = &lt_http_socket;
	lt_http_socket_boot(link, args);
	return LT_OK;
}

// ============================================================================
//  Public Functions
// ============================================================================

const char* lt_http_list() {
    return "socket";
}
