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

void test_decoded_5i() {
    char buffer[8];
    assert( ufr_dcr_msgpack_new(&g_link, 0) == UFR_OK );
    assert( ufr_boot_dcr(&g_link, NULL) == UFR_OK );

    {
        int recv[8];
        char send[] = {1,2,3,4,5,'\n'};
        ufr_write(&g_link, send, sizeof(send));
        ufr_recv(&g_link);
        for (int i=1; i<=5; i++) {
            int num;
            assert( ufr_get_type(&g_link) == 'i' );
            assert( ufr_get_nitems(&g_link) == 1 );
            // assert( ufr_get_raw_ptr(&g_link) != NULL );
            assert( ufr_get(&g_link, "i", &num) == 1 );
            assert( num == i );
        }
    }

    printf("OK - decoded 5 integers\n");
}

void test_decoded_3f() {
    char buffer[8];
    
    assert( ufr_dcr_msgpack_new(&g_link, 0) == UFR_OK );
    assert( ufr_boot_dcr(&g_link, NULL) == UFR_OK );

    {
        int recv[8];
        char send[] = {0xCA, 0x41, 0x28, 0, 0, 0xCA, 0x41, 0xA2, 0, 0, 0xCA, 0x41, 0xF1, 0, 0, '\n'};
        ufr_write(&g_link, send, sizeof(send));
        ufr_recv(&g_link);

        float num;
        assert( ufr_get_type(&g_link) == 'f' );
        assert( ufr_get(&g_link, "f", &num) == 1 );
        assert( num == 10.5 );

        assert( ufr_get_type(&g_link) == 'f' );
        assert( ufr_get(&g_link, "f", &num) == 1 );
        assert( num == 20.25 );

        assert( ufr_get_type(&g_link) == 'f' );
        assert( ufr_get(&g_link, "f", &num) == 1 );
        assert( num == 30.125 );
    }

    printf("OK - decoded 3 floats\n");
}


void test_decoded_2s() {
    char buffer[8];
    assert( ufr_dcr_msgpack_new(&g_link, 0) == UFR_OK );
    assert( ufr_boot_dcr(&g_link, NULL) == UFR_OK );

    {
        uint8_t send[] = {
            0xa6, 0x61, 0x62, 0x63, 0x31, 0x32, 0x33, 
            0xa6, 0x61, 0x62, 0x63, 0x31, 0x32, 0x33, 
            '\n'
        };
        ufr_write(&g_link, send, sizeof(send));
        ufr_recv(&g_link);

        char str[32];
        assert( ufr_get_type(&g_link) == 's' );
        assert( ufr_get(&g_link, "s", str) == 1 );
        assert( strcmp(str, "abc123") == 0);

        assert( ufr_get_type(&g_link) == 's' );
        assert( ufr_get(&g_link, "s", str) == 1 );
        assert( strcmp(str, "abc123") == 0 );
    }

    printf("OK - decoded 2 strings\n");
}

void test_decoded_array() {
    assert( ufr_dcr_msgpack_new(&g_link, 0) == UFR_OK );
    assert( ufr_boot_dcr(&g_link, NULL) == UFR_OK );

    {
        uint8_t send[] = {0x95, 0x13, 0x14, 0x1e, 0x28, 0x32, '\n'};
        ufr_write(&g_link, send, sizeof(send));
        ufr_recv(&g_link);

        assert( ufr_get_type(&g_link) == 'a' );
        int num;
        assert( ufr_get_enter(&g_link) == UFR_OK );
        assert( ufr_get(&g_link, "i", &num) == 1 );
        assert( num == 0x13 );
        assert( ufr_get(&g_link, "i", &num) == 1 );
        assert( num == 0x14 );
        assert( ufr_get(&g_link, "i", &num) == 1 );
        assert( num == 0x1e );
        assert( ufr_get(&g_link, "i", &num) == 1 );
        assert( num == 0x28 );
        assert( ufr_get(&g_link, "i", &num) == 1 );
        assert( num == 0x32 );
        assert( ufr_get(&g_link, "i", &num) == 0 );
        assert( num == 0 );
        assert( ufr_get_leave(&g_link) == UFR_OK );
    }

    printf("OK - decoded 1 array\n");
}


void test1() {
    char buffer[8];
    assert( ufr_dcr_msgpack_new(&g_link, 0) == UFR_OK );
    assert( ufr_boot_dcr(&g_link, NULL) == UFR_OK );

    {
        int recv[8];
        char send[] = {1,2,3,4,5,'\n'};
        ufr_write(&g_link, send, 7);
        ufr_get(&g_link, "^i", recv);
        for (int i=0; i<1; i++) {
            printf("%d ", recv[i]);
        }
        printf("\n");
    }

    ufr_close(&g_link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    g_link = ufr_new_pipe();

    test_decoded_5i();
    test_decoded_3f();
    test_decoded_2s();
    // test_decoded_array();

    ufr_close(&g_link);
	return 0;
}