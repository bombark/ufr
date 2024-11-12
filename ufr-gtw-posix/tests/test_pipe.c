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

int ufr_gtw_posix_new_pipe(link_t* link, int type);

// ============================================================================
//  Tests
// ============================================================================

void test_simple() {
    char buffer[8];
    link_t link;
    ufr_args_t args = {.text=""};
    assert( ufr_gtw_posix_new_pipe(&link, 0) == UFR_OK );
    assert( ufr_boot_gtw(&link, &args) == UFR_OK );
    assert( ufr_start(&link, 0, &args) == UFR_OK );    

    assert( ufr_write(&link, "Opa!", 4) == 4 );
    assert( ufr_recv(&link) == UFR_OK );

    assert( ufr_read(&link, buffer, sizeof(buffer)) == 4 );
    buffer[4] = '\0';
    assert( strcmp(buffer, "Opa!") == 0 );
    ufr_close(&link);
    printf("OK - test_simple\n");
}

void test_message_4kb() {
    char buffer[8];
    link_t link;
    ufr_args_t args = {.text=""};
    assert( ufr_gtw_posix_new_pipe(&link, 0) == UFR_OK );
    assert( ufr_boot_gtw(&link, &args) == UFR_OK );
    assert( ufr_start(&link, 0, &args) == UFR_OK );    

    // send a 1mb of data
    const int max = 4096;
    uint8_t buffer_snd[max];
    for (int i=0;i<max; i++) {
        buffer_snd[i] = (i%256);
    }
    assert( ufr_write(&link, buffer_snd, max) == max );

    // read and compare this 1mb
    uint8_t buffer_rcv[max];
    assert( ufr_recv(&link) == UFR_OK );
    assert( ufr_read(&link, buffer_rcv, max) == max );
    for (int i=0;i<max; i++) {
        assert( buffer_rcv[i] == (i%256) );
    }

    ufr_close(&link);
    printf("OK - test_message_1mb\n");
}

void test_new() {
    char buffer[8];
    link_t link = ufr_new("@new posix:pipe");
    assert( ufr_write(&link, "Opa!", 4) == 4 );
    assert( ufr_recv(&link) == UFR_OK );
    assert( ufr_read(&link, buffer, sizeof(buffer)) == 4 );
    buffer[4] = '\0';
    assert( strcmp(buffer, "Opa!") == 0 );
    ufr_close(&link);
    printf("OK - test_new\n");
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_simple();
    //test_new();
    test_message_4kb();
	return 0;
}