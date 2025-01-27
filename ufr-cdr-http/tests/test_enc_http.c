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

// ============================================================================
//  Tests
// ============================================================================

void test1() {
    char buffer[128];
    link_t link = ufr_new_pipe();
    
    ufr_enc_http_new(&link, 0);
    ufr_boot_enc(&link, NULL);

    {
        ufr_put(&link, "ss\n", "GET", "/");
        ufr_put_eof(&link);
        ufr_recv(&link);
        ufr_read(&link, buffer, sizeof(buffer));
        printf("%s\n", buffer);
    }

    ufr_close(&link);
}

void test2() {
    char buffer[128];
    link_t link = ufr_client("@new posix:stdout @coder http");

    ufr_put(&link, "ss\n\n", "GET", "/teste");


    ufr_close(&link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test2();
    return 0;
}