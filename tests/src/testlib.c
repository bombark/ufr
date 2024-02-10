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
//  Header
// ============================================================================

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ufr.h>

// ============================================================================
//  Tests
// ============================================================================

int test_publisher() {

}

// ============================================================================
//  Header
// ============================================================================

int main() {
    link_t link;
    // assert( ufr_sys_load_library(&link, "ufr_gtw_posix") == LT_OK );

    for (int i=0; i<10; i++) {
        const char* name = ufr_sys_lib_call_list(&link, i);
        if ( name == NULL ) {
            break;
        }
        printf("%s\n", name);
    }

    assert( ufr_sys_lib_call_new(&link, "file", NULL) == LT_OK );

    lt_args_t args = {.text="@path saida.txt"};
    ufr_boot_gtw(&link, &args);
    ufr_start(&link, &args);
    lt_write(&link, "opa\n", 4);
    lt_close(&link);
    return 0;
}