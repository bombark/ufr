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

int ufr_new_ecr_std_csv(link_t* link, const lt_args_t* args);

// ============================================================================
//  Tests
// ============================================================================

void test_simple() {
    link_t link = ufr_new("@new posix:pipe");
    lt_args_t args = {.text="@sep ;"};  
    ufr_new_ecr_std_csv(&link, &args);

    // test 1
    {
        char buffer[128];
        lt_put(&link, "iii\n", 10, 20, 30);
        int nbytes = lt_read(&link, buffer, sizeof(buffer));
        assert( nbytes == 9 );
        buffer[nbytes] = '\0';
        assert( strcmp(buffer, "10;20;30\n") == 0 );
    }

    // test 2
    {
        char buffer[128];
        lt_put(&link, "iii\n", 30, 20, 10);
        int nbytes = lt_read(&link, buffer, sizeof(buffer));
        assert( nbytes == 9 );
        buffer[nbytes] = '\0';
        assert( strcmp(buffer, "30;20;10\n") == 0 );

    }

    // test 3
    {
        char buffer[128];
        lt_put(&link, "sii\n", "string com espaco", 8759834, -712345);
        int nbytes = lt_read(&link, buffer, sizeof(buffer));
        assert( nbytes == 34 );
        buffer[nbytes] = '\0';
        assert( strcmp(buffer, "string com espaco;8759834;-712345\n") == 0 );
    }

    lt_close(&link);
}

void test_simple_2() {
    link_t link = ufr_new("@new posix:pipe");
    lt_args_t args = {.text="@sep ,"};  
    ufr_new_ecr_std_csv(&link, &args);

    // test 1
    {
        char buffer[128];
        lt_put(&link, "iii\n", 10, 20, 30);
        int nbytes = lt_read(&link, buffer, sizeof(buffer));
        assert( nbytes == 9 );
        buffer[nbytes] = '\0';
        assert( strcmp(buffer, "10,20,30\n") == 0 );
    }

    // test 2
    {
        char buffer[128];
        lt_put(&link, "iii\n", 30, 20, 10);
        int nbytes = lt_read(&link, buffer, sizeof(buffer));
        assert( nbytes == 9 );
        buffer[nbytes] = '\0';
        assert( strcmp(buffer, "30,20,10\n") == 0 );

    }

    // test 3
    {
        char buffer[128];
        lt_put(&link, "sii\n", "string com espaco", 8759834, -712345);
        int nbytes = lt_read(&link, buffer, sizeof(buffer));
        assert( nbytes == 34 );
        buffer[nbytes] = '\0';
        assert( strcmp(buffer, "string com espaco,8759834,-712345\n") == 0 );
    }

    lt_close(&link);
}


// ============================================================================
//  Main
// ============================================================================

int main() {
    test_simple();
    test_simple_2();
	return 0;
}