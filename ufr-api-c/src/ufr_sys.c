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

const char* g_ufr_sys_params = "@new zmq:socket @coder msgpack @host 127.0.0.1 @port 3000";

// ============================================================================
//  Public Functions
// ============================================================================

link_t ufr_sys_subscriber22(const char* name) {
    link_t link;

    // 
    link_t server = ufr_client(g_ufr_sys_params);
    ufr_put(&server, "ss\n", "open", name);

    // recv the answer
    int code;
    char sub_params[1024];
    if ( ufr_get(&server, "^is", &code, sub_params) > 0 ) {
        if ( code == 0 ) {
            ufr_log(&server, "got params: %s", sub_params);
            link = ufr_subscriber(sub_params);
        }
    }

    // close server link and return the opened link
    // ufr_close(&server);
    return link;
}

link_t ufr_sys_publisher22(const char* name, const char* params) {
    int code;
    char pub_params[1024];

    link_t server = ufr_client(g_ufr_sys_params);
    ufr_put(&server, "sss\n", "make", name, params);
    ufr_get(&server, "^is", &code, pub_params);

    link_t link;
    if ( code == UFR_OK ) {
        ufr_log(&server, "got params: %s", pub_params);
        link = ufr_publisher(pub_params);
    }
    return link;
}