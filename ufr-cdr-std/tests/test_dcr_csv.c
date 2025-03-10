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
#include <ufr_test.h>

int ufr_dcr_csv_new(link_t* link);

// ============================================================================
//  Tests
// ============================================================================

void test_simple() {
    link_t link;
    ufr_gtw_posix_new_pipe(&link, 0);
    ufr_boot_gtw(&link, NULL);
    ufr_args_t args = {.text="@sep ;"}; 
    ufr_dcr_csv_new(&link);
    ufr_boot_dcr(&link, &args);

    // test 1
    {
        int a=0,b=0,c=0;
        const char* buffer = "10;20;30\n";
        ufr_write(&link, buffer, strlen(buffer));
        ufr_get(&link, "^iii", &a, &b, &c);
        UFR_TEST_EQUAL_I32(a, 10);
        UFR_TEST_EQUAL_I32(b, 20);
        UFR_TEST_EQUAL_I32(c, 30);
    }

    // test 2
    {
        float a=0,b=0,c=0;
        const char* buffer = "10.5;20.125;30.5\n";
        ufr_write(&link, buffer, strlen(buffer));
        ufr_get(&link, "^fff", &a, &b, &c);
        UFR_TEST_EQUAL_F32(a, 10.5);
        UFR_TEST_EQUAL_F32(b, 20.125);
        UFR_TEST_EQUAL_F32(c, 30.5);
    }

    ufr_close(&link);
}

void test_simple_2() {
    link_t link;
    ufr_gtw_posix_new_pipe(&link, 0);
    ufr_boot_gtw(&link, NULL);

    ufr_args_t args = {.text="@sep ,"};  
    ufr_dcr_csv_new(&link);
    ufr_boot_dcr(&link, &args);

    // test 1
    {
        int a=0,b=0,c=0;
        const char* buffer = "10,20,30\n";
        ufr_write(&link, buffer, strlen(buffer));
        ufr_get(&link, "^iii", &a, &b, &c);
        UFR_TEST_EQUAL_I32(a, 10);
        UFR_TEST_EQUAL_I32(b, 20);
        UFR_TEST_EQUAL_I32(c, 30);
    }

    // test 2
    {
        float a=0,b=0,c=0;
        const char* buffer = "10.5,20.125,30.5\n";
        ufr_write(&link, buffer, strlen(buffer));
        ufr_get(&link, "^fff", &a, &b, &c);
        UFR_TEST_EQUAL_F32(a, 10.5);
        UFR_TEST_EQUAL_F32(b, 20.125);
        UFR_TEST_EQUAL_F32(c, 30.5);
    }

    // test 3
    {
        char text[512];
        const char* buffer = "abcde\n";
        ufr_write(&link, buffer, strlen(buffer));
        ufr_get(&link, "^s", text);
        UFR_TEST_EQUAL_STR(text, "abcde");
    }

    // test 4 - max size on buffer
    /*
    {
        char text[8];
        const char* buffer = "aaabbbcccddd\n";
        ufr_write(&link, buffer, strlen(buffer));
        ufr_get(&link, "^s", text);
        assert( strcmp(text, "aaabbbc") == 0 );
    }
    */

    ufr_close(&link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_simple();
    test_simple_2();
    ufr_test_print_result();
	return 0;
}