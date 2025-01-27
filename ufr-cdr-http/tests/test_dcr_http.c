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
//  HEADER
// ============================================================================

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ufr.h>

#include "test.h"

link_t g_link;

// ============================================================================
//  Tests
// ============================================================================

void test_decoded_req() {
    char buffer[64];
    ufr_dcr_http_req_new(&g_link, 0);
    ufr_boot_dcr(&g_link, NULL);

    {
        char send[] = "GET /api?param1=value1&param2=value2 HTTP/1.1\r\n"
            "Host: example.com\r\n"
            "User-Agent: curl/7.68.0\r\n"
            "Accept: */*\r\n"
            "\r\n";

        ufr_write(&g_link, send, sizeof(send));
        ufr_recv(&g_link);
        
        ufr_get_str(&g_link, buffer, 64);
        printf("%s\n", buffer);

        ufr_get_str(&g_link, buffer, 64);
        printf("%s\n", buffer);

        for (int i=0; i<10; i++) {
            ufr_get_str(&g_link, buffer, 64);
            printf("%s\n", buffer);
        }



        // ufr_get_type(&g_link)
        // ufr_get_nitems(&g_link);
        // ufr_get_nitems(&g_link);
    }
}

void test_decoded_ans() {
    char buffer[64];
    ufr_dcr_http_ans_new(&g_link, 0);
    ufr_boot_dcr(&g_link, NULL);

    {
        char send[] = "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: 13\r\n"
            "\r\n"
            "Hello, World!";

        ufr_write(&g_link, send, sizeof(send));
        ufr_recv(&g_link);
        
        ufr_get_str(&g_link, buffer, 64);
        printf("%s\n", buffer);

        ufr_get_str(&g_link, buffer, 64);
        printf("%s\n", buffer);

        ufr_get_str(&g_link, buffer, 64);
        printf("%s\n", buffer);

        for (int i=0; i<10; i++) {
            ufr_get_str(&g_link, buffer, 64);
            printf("%s\n", buffer);
        }

    }
}


// ============================================================================
//  Main
// ============================================================================

int main() {
    g_link = ufr_new_pipe();

    // test_decoded_5i();
    test_decoded_ans();

    ufr_close(&g_link);
	return 0;
}