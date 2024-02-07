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

int ufr_gtw_posix_new_file(link_t* link, int type);

// ============================================================================
//  Tests
// ============================================================================

void test_write() {
    link_t link;
    lt_args_t args = {.text="@path ./teste.txt"};
    assert( ufr_gtw_posix_new_file(&link, LT_START_PUBLISHER) == LT_OK );
    assert( ufr_boot(&link, &args) == LT_OK );
    assert( ufr_start(&link, &args) == LT_OK );
    assert( lt_write(&link, "OPA\n", 4) == 4 );
    // lt_stop(&link);
    lt_close(&link);
}

void test_read() {
    char buffer[8];
    link_t link;
    lt_args_t args = {.text="@path ./teste.txt"};
    assert( ufr_gtw_posix_new_file(&link, LT_START_SUBSCRIBER) == LT_OK );
    assert( ufr_boot(&link, &args) == LT_OK );
    assert( ufr_start(&link, &args) == LT_OK );    
    assert( lt_read(&link, buffer, 8) == 4 );
    lt_close(&link);
}

void test_new() {
    link_t link = ufr_new("@new posix:file @path ./teste.txt");
    assert( ufr_start_publisher(&link, NULL) == LT_OK );
    lt_close(&link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_write();
    test_read();
    // test_new();
	return 0;
}