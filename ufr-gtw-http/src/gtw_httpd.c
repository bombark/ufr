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
#include <string.h>
#include <microhttpd.h>
#include <ufr.h>

struct MHD_Daemon* g_daemon;

#define PAGE "<html><head><title>libmicrohttpd demo</title>" \
             "</head><body>libmicrohttpd demo</body></html>"

volatile int g_wait;
link_t g_ahc_input;
link_t g_ahc_output;

static 
enum MHD_Result ahc_echo (
    void* cls,
    struct MHD_Connection* connection,
    const char* url,
    const char* method,
    const char* version,
    const char* upload_data,
    size_t* upload_data_size,
    void** ptr
) {
printf("opa\n");
    static int dummy;
    if (&dummy != *ptr) {
        *ptr = &dummy;
        return MHD_YES;
    }

    g_wait = 1;

    char buffer[512];
    size_t size = lt_read(&g_ahc_input, buffer, 512);
    buffer[size] = '\0';
// printf("%d %s\n", size, buffer);

    // send the answer
    struct MHD_Response* response;
    response = MHD_create_response_from_buffer(4, buffer, MHD_RESPMEM_PERSISTENT);
printf("%d %s\n", size, buffer);
    int ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);
    return ret;
}



// ============================================================================
//  File
// ============================================================================

static
int gw_httpd_socket_type(const link_t* link) {
	return 0;
}

static
int gw_httpd_socket_state(const link_t* link){
	return 0;
}

static
size_t gw_httpd_socket_size(const link_t* link, int type){
	return 0;
}

static
int gw_httpd_socket_boot(link_t* link, const ufr_args_t* args) {

	return UFR_OK;
}

static
int gw_httpd_socket_start(link_t* link, int type, const ufr_args_t* args) {

    if ( type == UFR_START_SERVER ) {
        g_wait = 0;
        // lt_new_ptr(&g_ahc_input, "@new posix:pipe");
        // lt_new_ptr(&g_ahc_output, "@new posix:pipe");

        g_daemon = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, 2000,
            NULL,
            NULL,
            &ahc_echo,
            NULL,
            MHD_OPTION_END);

    }

	return 0;
}

static
void gw_httpd_socket_stop(link_t* link, int type) {
    MHD_stop_daemon(g_daemon);
}

static
int gw_httpd_socket_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t gw_httpd_socket_read(link_t* link, char* buffer, size_t length) {
    return 0; // lt_read(&g_ahc_output, buffer, length);
}

static
size_t gw_httpd_socket_write(link_t* link, const char* buffer, size_t length) {
    return 0; // lt_write(&g_ahc_input, buffer, length);
}

static
void gw_httpd_socket_recv(link_t* link) {
    while (g_wait == 0)
        ;
    g_wait = 0;
}

static
ufr_gtw_api_t lt_httpd_socket = {
	.type = gw_httpd_socket_type,
	.state = gw_httpd_socket_state,
	.size = gw_httpd_socket_size,
	.boot = gw_httpd_socket_boot,
	.start = gw_httpd_socket_start,
	.stop = gw_httpd_socket_stop,
	.copy = gw_httpd_socket_copy,
	.read = gw_httpd_socket_read,
	.write = gw_httpd_socket_write,
    .recv = gw_httpd_socket_recv,
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_http_socket_new(link_t* link, int type) {
	link->gtw_api = &lt_httpd_socket;
	return UFR_OK;
}