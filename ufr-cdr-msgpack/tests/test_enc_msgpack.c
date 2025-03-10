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
    char buffer[8];
    link_t link = ufr_new_pipe();
    
    assert( ufr_enc_msgpack_new(&link, 0) == UFR_OK );
    assert( ufr_boot_enc(&link, NULL) == UFR_OK );
    
    {
        ufr_put(&link, "iii\n", 10, 20, 30);
        assert( ufr_recv(&link) == UFR_OK );
        assert( ufr_read(&link, buffer, sizeof(buffer)) == 3 );
        assert( buffer[0] == 10 );
        assert( buffer[1] == 20 );
        assert( buffer[2] == 30 );
    }

    {
        ufr_put(&link, "s\n", "hello");
        assert( ufr_recv(&link) == UFR_OK );
        assert( ufr_read(&link, buffer, sizeof(buffer)) == 6 );
        assert( strncmp(&buffer[2], "hello", 4) );
    }

    /*{
        int vet[5] = {1,2,3,4,5};
        ufr_put(&link, "ai\n", 5, vet);
        assert( ufr_recv(&link) == UFR_OK );
        int c1 = ufr_read(&link, buffer, sizeof(buffer));
        printf("%d\n", c1);
        assert( c1 == 6 );
        assert( buffer[0] == -107 );
        assert( buffer[1] == 1 );
        assert( buffer[2] == 2 );
        assert( buffer[3] == 3 );
        assert( buffer[4] == 4 );
        assert( buffer[5] == 5 );
    }*/

    ufr_close(&link);
    printf("test1 - OK\n");
}

void test_encoder_array() {
    link_t link = ufr_new_pipe();
    assert( ufr_enc_msgpack_new(&link, 0) == UFR_OK );
    assert( ufr_boot_enc(&link, NULL) == UFR_OK );
    
    {
        int vet[5] = {20,21,22,23,24};
        // ufr_put(&link, "ai\n", 5, vet);
        ufr_put_i32(&link, vet, 5);
        ufr_put(&link, "\n");
        uint8_t buffer[8];
        assert( ufr_read(&link, buffer, sizeof(buffer)) == 7 );
        
        // assert( ufr_read(&link, buffer, sizeof(buffer)) == 5 );
        assert( buffer[0] == 0x95 );
        assert( buffer[1] == 20 );
        assert( buffer[2] == 21 );
        assert( buffer[3] == 22 );
        assert( buffer[4] == 23 );
        assert( buffer[5] == 24 );
    }

    ufr_close(&link);
    printf("encoded 1 array - OK\n");
}

void show_encoder_bytes() {
    link_t link = ufr_new_pipe();
    assert( ufr_enc_msgpack_new(&link, 0) == UFR_OK );
    assert( ufr_boot_enc(&link, NULL) == UFR_OK );

    ufr_put_enter(&link, 5);
    ufr_put(&link, "iiiii", 10, 20, 30, 40, 50);
    ufr_put_leave(&link);
    ufr_put(&link, "\n");

    /* ufr_enter_array(&link, 3);
    for (int i=0; i<3; i++) {
        ufr_put(&link, "i", i);
    }
    ufr_leave_array(&link);
    ufr_put(&link, "\n");*/

    uint8_t buffer[1024];
    size_t read = ufr_read(&link, buffer, 1024);
    for (int i=0; i<read; i++ ){
        printf("%x ", buffer[i]);
    }
    printf("\n");

    ufr_close(&link);
}



// ============================================================================
//  Main
// ============================================================================

int main() {
    test1();
    // test_encoder_array();
    // show_encoder_bytes();
    return 0;
}