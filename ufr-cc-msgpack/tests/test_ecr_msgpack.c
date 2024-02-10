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

int ufr_ecr_msgpack_new_obj(link_t* link, const int type);

// ============================================================================
//  Tests
// ============================================================================

void test1() {
    char buffer[8];
    link_t link = ufr_new("@new posix:pipe");
    assert( ufr_ecr_msgpack_new_obj(&link, 0) == LT_OK );
    assert( ufr_boot_ecr(&link, NULL) == LT_OK );
    
    {
        lt_put(&link, "iii\n", 10, 20, 30);
        assert( lt_read(&link, buffer, sizeof(buffer)) == 3 );
        assert( buffer[0] == 10 );
        assert( buffer[1] == 20 );
        assert( buffer[2] == 30 );
    }

    {
        lt_put(&link, "s\n", "hello");
        assert( lt_read(&link, buffer, sizeof(buffer)) == 6 );
        assert( strncmp(&buffer[2], "hello", 4) );
    }

    {
        int vet[5] = {1,2,3,4,5};
        lt_put(&link, "ai\n", 5, vet);
        assert( lt_read(&link, buffer, sizeof(buffer)) == 6 );
        assert( buffer[0] == -107 );
        assert( buffer[1] == 1 );
        assert( buffer[2] == 2 );
        assert( buffer[3] == 3 );
        assert( buffer[4] == 4 );
        assert( buffer[5] == 5 );
    }

    lt_close(&link);
}


void test2() {
    link_t link = ufr_publisher("@new posix:file @path saida.txt");
    assert( ufr_ecr_msgpack_new_obj(&link, 0) == LT_OK );
    assert( ufr_boot_ecr(&link, NULL) == LT_OK );

    ufr_put(&link, "iii", 10, 20, 30);
    ufr_enter_array(&link, 3);
    for (int i=0; i<3; i++) {
        ufr_put(&link, "i", i);
    }
    ufr_leave_array(&link);
    ufr_put(&link, "\n");


    ufr_close(&link);
}



// ============================================================================
//  Main
// ============================================================================

int main() {
    test1();
    // test2();
	return 0;
}